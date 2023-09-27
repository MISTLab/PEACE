# CLIP Interrogator for Aerial Robotics by [@HaechanMarkBong](https://github.com/HaeChanBong/clip-interrogator-aerial-robotics/tree/HaechanMarkBong-patch-1)

Built on top of CLIP Interrogator by [@pharmapsychotic](https://twitter.com/pharmapsychotic)

Generate prompts suited for aerial robotics (real and simlation) to be used for CLIPSeg.

## About

The **CLIP Interrogator** is a prompt engineering tool that combines OpenAI's [CLIP](https://openai.com/blog/clip/) and Salesforce's [BLIP](https://blog.salesforceairesearch.com/blip-bootstrapping-language-image-pretraining/) to optimize text prompts to match a given image. Use the resulting prompts with text-to-image models like [CLIPSeg]([https://github.com/CompVis/stable-diffusion](https://github.com/timojl/clipseg)).


You can then use it in your script
```python
from PIL import Image
from clip_interrogator import Config, Interrogator
image = Image.open(image_path).convert('RGB')
ci = Interrogator(Config(clip_model_name="ViT-L-14/openai"))
print(ci.interrogate(image))
```

CLIP Interrogator uses OpenCLIP which supports many different pretrained CLIP models. For the best prompts for 
CLIPSeg use `ViT-L-14/openai` for clip_model_name.

## Configuration

The `Config` object lets you configure CLIP Interrogator's processing. 
* `clip_model_name`: which of the OpenCLIP pretrained CLIP models to use
* `cache_path`: path where to save precomputed text embeddings
* `download_cache`: when True will download the precomputed embeddings from huggingface
* `chunk_size`: batch size for CLIP, use smaller for lower VRAM
* `quiet`: when True no progress bars or text output will be displayed

On systems with low VRAM you can call `config.apply_low_vram_defaults()` to reduce the amount of VRAM needed (at the cost of some speed and quality). The default settings use about 6.3GB of VRAM and the low VRAM settings use about 2.7GB.

See the [run_cli.py](https://github.com/pharmapsychotic/clip-interrogator/blob/main/run_cli.py) and [run_gradio.py](https://github.com/pharmapsychotic/clip-interrogator/blob/main/run_gradio.py) for more examples on using Config and Interrogator classes.


## Ranking against your own list of terms (requires version 0.6.0)

```python
from clip_interrogator import Config, Interrogator, LabelTable, load_list
from PIL import Image

ci = Interrogator(Config(blip_model_type=None))
image = Image.open(image_path).convert('RGB')
table = LabelTable(load_list('terms.txt'), 'terms', ci)
best_match = table.rank(ci.image_to_features(image), top_count=1)[0]
print(best_match)
```
