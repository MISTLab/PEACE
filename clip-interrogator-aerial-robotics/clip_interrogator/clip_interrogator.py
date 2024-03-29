import hashlib
import math
import numpy as np
import open_clip
import os
import requests
import time
import torch

from dataclasses import dataclass
from PIL import Image
from transformers import AutoProcessor, AutoModelForCausalLM, BlipForConditionalGeneration, Blip2ForConditionalGeneration
from tqdm import tqdm
from typing import List, Optional
from safetensors.numpy import load_file, save_file

CAPTION_MODELS = {
    'blip-base': 'Salesforce/blip-image-captioning-base',   # 990MB
    'blip-large': 'Salesforce/blip-image-captioning-large', # 1.9GB
    'blip2-2.7b': 'Salesforce/blip2-opt-2.7b',              # 15.5GB
    'blip2-flan-t5-xl': 'Salesforce/blip2-flan-t5-xl',      # 15.77GB
    'git-large-coco': 'microsoft/git-large-coco',           # 1.58GB
}

CACHE_URL_BASE = 'https://huggingface.co/pharma/ci-preprocess/resolve/main/'

@dataclass 
class Config:
    # Models can optionally be passed in directly
    caption_model = None
    caption_processor = None
    clip_model = None
    clip_preprocess = None

    # BLIP settings
    caption_max_length: int = 32
    caption_model_name: Optional[str] = 'blip-base' # Use a key from CAPTION_MODELS or None
    caption_offload: bool = True

    # CLIP settings
    clip_model_name: str = 'ViT-L-14/openai'
    clip_model_path: Optional[str] = None
    clip_offload: bool = True

    # Interrogator settings
    cache_path: str = 'cache'   # Path to store cached text embeddings
    download_cache: bool = True # When true, cached embeds are downloaded from huggingface
    chunk_size: int = 512    # Batch size for CLIP, use smaller for lower VRAM
    data_path: str = os.path.join(os.path.dirname(__file__), 'data')
    device: str = ("mps" if torch.backends.mps.is_available() else "cuda" if torch.cuda.is_available() else "cpu")
    env_intermediate_count: int = 512
    quiet: bool = False # When quiet progress bars are not shown

    def apply_low_vram_defaults(self):
        self.caption_model_name = 'blip-base'
        self.caption_offload = True
        self.clip_offload = True
        self.chunk_size = 256
        self.env_intermediate_count = 256

class Interrogator():
    def __init__(self, config: Config):
        self.config = config
        self.device = config.device
        self.dtype = torch.float16 if self.device == 'cuda' else torch.float32
        self.caption_offloaded = True
        self.clip_offloaded = True
        self.load_caption_model()
        self.load_clip_model()

    def load_caption_model(self):
        if self.config.caption_model is None and self.config.caption_model_name:
            if not self.config.quiet:
                print(f"Loading caption model {self.config.caption_model_name}...")

            model_path = CAPTION_MODELS[self.config.caption_model_name]
            if self.config.caption_model_name.startswith('git-'):
                caption_model = AutoModelForCausalLM.from_pretrained(model_path, torch_dtype = torch.float32)
            elif self.config.caption_model_name.startswith('blip2-'):
                caption_model = Blip2ForConditionalGeneration.from_pretrained(model_path, torch_dtype = self.dtype)
            else:
                caption_model = BlipForConditionalGeneration.from_pretrained(model_path, torch_dtype = self.dtype)
            self.caption_processor = AutoProcessor.from_pretrained(model_path)

            caption_model.eval()
            if not self.config.caption_offload:
                caption_model = caption_model.to(self.config.device)
            self.caption_model = caption_model
        else:
            self.caption_model = self.config.caption_model
            self.caption_processor = self.config.caption_processor

    def load_clip_model(self):
        start_time = time.time()
        config = self.config

        clip_model_name, clip_model_pretrained_name = config.clip_model_name.split('/', 2)

        if config.clip_model is None:
            if not config.quiet:
                print(f"Loading CLIP model {config.clip_model_name}...")

            self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms(
                clip_model_name, 
                pretrained = clip_model_pretrained_name, 
                precision = 'fp16' if config.device == 'cuda' else 'fp32',
                device = config.device,
                jit=  False,
                cache_dir = config.clip_model_path
            )
            self.clip_model.eval()
        else:
            self.clip_model = config.clip_model
            self.clip_preprocess = config.clip_preprocess
        self.tokenize = open_clip.get_tokenizer(clip_model_name)

        positives = ["grass", "backyard", "frontyard", "courtyard", "lawn", "greenery", "garden",
                     "sports-field", "terrain", "field", "park", "open-area", "open-space",
                     "trail", "sidewalk", "driveway", "terrace", "porch", "patio"]
        positive_list = [positive for positive in positives]
        positive_list.extend(["of "+positive for positive in positives])

        raw_frames = load_list(config.data_path, 'frames.txt')

        self._prepare_clip()
        self.frames = LabelTable(raw_frames, "frames", self)
        self.envs = LabelTable(load_list(config.data_path, 'envs.txt'), "envs", self)
        self.res = LabelTable(load_list(config.data_path, 'res.txt'), "res", self)
        self.contexts = LabelTable(load_list(config.data_path, 'contexts.txt'), "contexts", self)
        self.positives = LabelTable(positive_list, "positives", self)
        self.aerials = LabelTable(load_list(config.data_path, 'aerials.txt'), "aerials", self)

        end_time = time.time()
        if not config.quiet:
            print(f"Loaded CLIP model and data in {end_time - start_time: .2f} seconds.")

    def chain(
        self, 
        image_features: torch.Tensor, 
        phrases: List[str], 
        best_prompt: str = "", 
        best_sim: float = 0, 
        min_count: int = 8,
        max_count: int = 32, 
        desc = "Chaining", 
        reverse: bool = False
    ) -> str:
        self._prepare_clip()

        phrases = set(phrases)
        if not best_prompt:
            best_prompt = self.rank_top(image_features, [f for f in phrases], reverse = reverse)
            best_sim = self.similarity(image_features, best_prompt)
            phrases.remove(best_prompt)
        curr_prompt, curr_sim = best_prompt, best_sim
        
        def check(addition: str, idx: int) -> bool:
            nonlocal best_prompt, best_sim, curr_prompt, curr_sim
            prompt = curr_prompt + ", " + addition
            sim = self.similarity(image_features, prompt)
            if reverse:
                sim = -sim
            
            if sim > best_sim:
                best_prompt, best_sim = prompt, sim
            if sim > curr_sim or idx < min_count:
                curr_prompt, curr_sim = prompt, sim
                return True
            return False

        for idx in tqdm(range(max_count), desc = desc, disable = self.config.quiet):
            best = self.rank_top(image_features, [f"{curr_prompt}, {f}" for f in phrases], reverse = reverse)
            flave = best[len(curr_prompt) + 2:]
            if not check(flave, idx):
                break
            if _prompt_at_max_len(curr_prompt, self.tokenize):
                break
            phrases.remove(flave)

        return best_prompt

    def generate_caption(self, pil_image: Image) -> str:
        assert self.caption_model is not None, "No caption model loaded."
        self._prepare_caption()
        inputs = self.caption_processor(images = pil_image, return_tensors = "pt").to(self.device)
        if not self.config.caption_model_name.startswith('git-'):
            inputs = inputs.to(self.dtype)
        tokens = self.caption_model.generate(**inputs, max_new_tokens = self.config.caption_max_length)
        return self.caption_processor.batch_decode(tokens, skip_special_tokens = True)[0].strip()

    def image_to_features(self, image: Image) -> torch.Tensor:
        self._prepare_clip()
        images = self.clip_preprocess(image).unsqueeze(0).to(self.device)
        with torch.no_grad(), torch.cuda.amp.autocast():
            image_features = self.clip_model.encode_image(images)
            image_features /= image_features.norm(dim = -1, keepdim = True)
        return image_features

    def interrogate_classic(self, image: Image, max_envs: int = 3, caption: Optional[str] = None) -> str:
        """Classic mode creates a prompt in a standard format first describing the image, 
        then listing the frame, positive, context, and env text modifiers."""
        caption = caption or self.generate_caption(image)
        image_features = self.image_to_features(image)

        res = self.res.rank(image_features, 1)[0]
        frame = self.frames.rank(image_features, 1)[0]
        positive = self.positives.rank(image_features, 1)[0]
        context = self.contexts.rank(image_features, 1)[0]
        aerial = self.aerials.rank(image_features, 1)[0]
        env = ", ".join(self.envs.rank(image_features, max_envs))

        if caption.startswith(res):
            prompt = f"A {frame} of {{}} in {env}."
        else:
            prompt = f"A {res} {frame} of {{}} in {env}."

        return _truncate_to_fit(prompt, self.tokenize)

    def interrogate_fast(self, image: Image, max_envs: int=32, caption: Optional[str]=None) -> str:
        """Fast mode simply adds the top ranked terms after a caption. It generally results in 
        better similarity between generated prompt and image than classic mode, but the prompts
        are less readable."""
        caption = caption or self.generate_caption(image)
        image_features = self.image_to_features(image)
        merged = _merge_tables([self.frames, self.envs, self.res, self.contexts, self.positives], self)
        tops = merged.rank(image_features, max_envs)
        return _truncate_to_fit(caption + ", " + ", ".join(tops), self.tokenize)

    def interrogate_aerial(self, image: Image, max_envs: int = 32) -> str:
        """aerial mode chains together the most dissimilar terms to the image. It can be used
        to help build a aerial prompt to pair with the regular positive prompt and often 
        improve the results of generated images particularly with CLIPSeg."""
        image_features = self.image_to_features(image)
        env = self.envs.rank(image_features, self.config.env_intermediate_count, reverse = True)
        env = env + self.aerials.labels
        return self.chain(image_features, env, max_count = max_envs, reverse = True, desc = "aerial chain")

    def interrogate(self, image: Image, min_envs: int = 8, max_envs: int = 32, caption: Optional[str] = None) -> str:
        caption = caption or self.generate_caption(image)
        image_features = self.image_to_features(image)

        merged = _merge_tables([self.frames, self.envs, self.res, self.contexts, self.positives], self)
        env = merged.rank(image_features, self.config.env_intermediate_count)
        best_prompt, best_sim = caption, self.similarity(image_features, caption)
        best_prompt = self.chain(image_features, env, best_prompt, best_sim, min_count = min_envs, max_count = max_envs, desc = "env chain")

        fast_prompt = self.interrogate_fast(image, max_envs, caption = caption)
        classic_prompt = self.interrogate_classic(image, max_envs, caption = caption)
        candidates = [caption, classic_prompt, fast_prompt, best_prompt]
        return candidates[np.argmax(self.similarities(image_features, candidates))]

    def rank_top(self, image_features: torch.Tensor, text_array: List[str], reverse: bool = False) -> str:
        self._prepare_clip()
        text_tokens = self.tokenize([text for text in text_array]).to(self.device)
        with torch.no_grad(), torch.cuda.amp.autocast():
            text_features = self.clip_model.encode_text(text_tokens)
            text_features /= text_features.norm(dim = -1, keepdim = True)
            similarity = text_features @ image_features.T
            if reverse:
                similarity = -similarity
        return text_array[similarity.argmax().item()]

    def similarity(self, image_features: torch.Tensor, text: str) -> float:
        self._prepare_clip()
        text_tokens = self.tokenize([text]).to(self.device)
        with torch.no_grad(), torch.cuda.amp.autocast():
            text_features = self.clip_model.encode_text(text_tokens)
            text_features /= text_features.norm(dim = -1, keepdim = True)
            similarity = text_features @ image_features.T
        return similarity[0][0].item()

    def similarities(self, image_features: torch.Tensor, text_array: List[str]) -> List[float]:
        self._prepare_clip()
        text_tokens = self.tokenize([text for text in text_array]).to(self.device)
        with torch.no_grad(), torch.cuda.amp.autocast():
            text_features = self.clip_model.encode_text(text_tokens)
            text_features /= text_features.norm(dim = -1, keepdim = True)
            similarity = text_features @ image_features.T
        return similarity.T[0].tolist()

    def _prepare_caption(self):
        if self.config.clip_offload and not self.clip_offloaded:
            self.clip_model = self.clip_model.to('cpu')
            self.clip_offloaded = True
        if self.caption_offloaded:
            self.caption_model = self.caption_model.to(self.device)
            self.caption_offloaded = False

    def _prepare_clip(self):
        if self.config.caption_offload and not self.caption_offloaded:
            self.caption_model = self.caption_model.to('cpu')
            self.caption_offloaded = True
        if self.clip_offloaded:
            self.clip_model = self.clip_model.to(self.device)
            self.clip_offloaded = False

class LabelTable():
    def __init__(self, labels:List[str], desc:str, ci: Interrogator):
        clip_model, config = ci.clip_model, ci.config
        self.chunk_size = config.chunk_size
        self.config = config
        self.device = config.device
        self.embeds = []
        self.labels = labels
        self.tokenize = ci.tokenize

        hash = hashlib.sha256(",".join(labels).encode()).hexdigest()
        sanitized_name = self.config.clip_model_name.replace('/', '_').replace('@', '_')
        self._load_cached(desc, hash, sanitized_name)

        if len(self.labels) != len(self.embeds):
            self.embeds = []
            chunks = np.array_split(self.labels, max(1, len(self.labels) / config.chunk_size))
            for chunk in tqdm(chunks, desc = f"Preprocessing {desc}" if desc else None, disable = self.config.quiet):
                text_tokens = self.tokenize(chunk).to(self.device)
                with torch.no_grad(), torch.cuda.amp.autocast():
                    text_features = clip_model.encode_text(text_tokens)
                    text_features /= text_features.norm(dim = -1, keepdim = True)
                    text_features = text_features.half().cpu().numpy()
                for i in range(text_features.shape[0]):
                    self.embeds.append(text_features[i])

            if desc and self.config.cache_path:
                os.makedirs(self.config.cache_path, exist_ok = True)
                cache_filepath = os.path.join(self.config.cache_path, f"{sanitized_name}_{desc}.safetensors")
                tensors = {
                    "embeds": np.stack(self.embeds),
                    "hash": np.array([ord(c) for c in hash], dtype = np.int8)
                }
                save_file(tensors, cache_filepath)

        if self.device == 'cpu' or self.device == torch.device('cpu'):
            self.embeds = [e.astype(np.float32) for e in self.embeds]

    def _load_cached(self, desc:str, hash:str, sanitized_name:str) -> bool:
        if self.config.cache_path is None or desc is None:
            return False

        cached_safetensors = os.path.join(self.config.cache_path, f"{sanitized_name}_{desc}.safetensors")

        if self.config.download_cache and not os.path.exists(cached_safetensors):
            download_url = CACHE_URL_BASE + f"{sanitized_name}_{desc}.safetensors"
            try:
                os.makedirs(self.config.cache_path, exist_ok=True)
                _download_file(download_url, cached_safetensors, quiet=self.config.quiet)
            except Exception as e:
                print(f"Failed to download {download_url}")
                print(e)
                return False                

        if os.path.exists(cached_safetensors):
            try:
                tensors = load_file(cached_safetensors)
            except Exception as e:
                print(f"Failed to load {cached_safetensors}")
                print(e)
                return False
            if 'hash' in tensors and 'embeds' in tensors:
                if np.array_equal(tensors['hash'], np.array([ord(c) for c in hash], dtype=np.int8)):
                    self.embeds = tensors['embeds']
                    if len(self.embeds.shape) == 2:
                        self.embeds = [self.embeds[i] for i in range(self.embeds.shape[0])]
                    return True

        return False
    
    def _rank(self, image_features: torch.Tensor, text_embeds: torch.Tensor, top_count: int = 1, reverse: bool = False) -> str:
        top_count = min(top_count, len(text_embeds))
        text_embeds = torch.stack([torch.from_numpy(t) for t in text_embeds]).to(self.device)
        with torch.cuda.amp.autocast():
            similarity = image_features @ text_embeds.T
            if reverse:
                similarity = -similarity
        _, top_labels = similarity.float().cpu().topk(top_count, dim = -1)
        return [top_labels[0][i].numpy() for i in range(top_count)]

    def rank(self, image_features: torch.Tensor, top_count: int = 1, reverse: bool=False) -> List[str]:
        if len(self.labels) <= self.chunk_size:
            tops = self._rank(image_features, self.embeds, top_count = top_count, reverse = reverse)
            return [self.labels[i] for i in tops]

        num_chunks = int(math.ceil(len(self.labels) / self.chunk_size))
        keep_per_chunk = int(self.chunk_size / num_chunks)

        top_labels, top_embeds = [], []
        for chunk_idx in tqdm(range(num_chunks), disable = self.config.quiet):
            start = chunk_idx * self.chunk_size
            stop = min(start + self.chunk_size, len(self.embeds))
            tops = self._rank(image_features, self.embeds[start:stop], top_count = keep_per_chunk, reverse = reverse)
            top_labels.extend([self.labels[start + i] for i in tops])
            top_embeds.extend([self.embeds[start + i] for i in tops])

        tops = self._rank(image_features, top_embeds, top_count = top_count)
        return [top_labels[i] for i in tops]

def _download_file(url: str, filepath: str, chunk_size: int = 4 * 1024 * 1024, quiet: bool = False):
    r = requests.get(url, stream = True)
    if r.status_code != 200:
        return

    file_size = int(r.headers.get("Content-Length", 0))
    filename = url.split("/")[-1]
    progress = tqdm(total = file_size, unit = "B", unit_scale = True, desc = filename, disable = quiet)
    with open(filepath, "wb") as f:
        for chunk in r.iter_content(chunk_size = chunk_size):
            if chunk:
                f.write(chunk)
                progress.update(len(chunk))
    progress.close()

def _merge_tables(tables: List[LabelTable], ci: Interrogator) -> LabelTable:
    m = LabelTable([], None, ci)
    for table in tables:
        m.labels.extend(table.labels)
        m.embeds.extend(table.embeds)
    return m

def _prompt_at_max_len(text: str, tokenize) -> bool:
    tokens = tokenize([text])
    return tokens[0][-1] != 0

def _truncate_to_fit(text: str, tokenize) -> str:
    parts = text.split(', ')
    new_text = parts[0]
    for part in parts[1:]:
        if _prompt_at_max_len(new_text + part, tokenize):
            break
        new_text += ', ' + part
    return new_text

def list_caption_models() -> List[str]:
    return list(CAPTION_MODELS.keys())

def list_clip_models() -> List[str]:
    return ['/'.join(x) for x in open_clip.list_pretrained()]

def load_list(data_path: str, filename: Optional[str] = None) -> List[str]:
    """Load a list of strings from a file."""
    if filename is not None:
        data_path = os.path.join(data_path, filename)
    with open(data_path, 'r', encoding = 'utf-8', errors = 'replace') as f:
        items = [line.strip() for line in f.readlines()]
    return items
