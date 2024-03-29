#!/usr/bin/env python3
import argparse
import torch
from clip_interrogator import Config, Interrogator, list_caption_models, list_clip_models

try:
    import gradio as gr
except ImportError:
    print("Gradio is not installed, please install it with 'pip install gradio'")
    exit(1)

parser = argparse.ArgumentParser()
parser.add_argument("--lowvram", action = 'store_true', help = "Optimize settings for low VRAM")
parser.add_argument('-s', '--share', action = 'store_true', help = 'Create a public link')
args = parser.parse_args()

if not torch.cuda.is_available():
    print("CUDA is not available, using CPU. Warning: this will be very slow!")

config = Config(cache_path = "cache")
if args.lowvram:
    config.apply_low_vram_defaults()
ci = Interrogator(config)

def image_analysis(image, clip_model_name):
    if clip_model_name != ci.config.clip_model_name:
        ci.config.clip_model_name = clip_model_name
        ci.load_clip_model()

    image = image.convert('RGB')
    image_features = ci.image_to_features(image)

    top_res = ci.res.rank(image_features, 5)
    top_frames = ci.frames.rank(image_features, 5)
    top_contexts = ci.contexts.rank(image_features, 5)
    top_aerials = ci.aerials.rank(image_features, 5)
    top_positives = ci.positives.rank(image_features, 5)
    top_envs = ci.envs.rank(image_features, 5)

    res_ranks = {res: sim for res, sim in zip(top_res, ci.similarities(image_features, top_res))}
    frame_ranks = {frame: sim for frame, sim in zip(top_frames, ci.similarities(image_features, top_frames))}
    context_ranks = {context: sim for context, sim in zip(top_contexts, ci.similarities(image_features, top_contexts))}
    aerial_ranks = {aerial: sim for aerial, sim in zip(top_aerials, ci.similarities(image_features, top_aerials))}
    positive_ranks = {positive: sim for positive, sim in zip(top_positives, ci.similarities(image_features, top_positives))}
    env_ranks = {env: sim for env, sim in zip(top_envs, ci.similarities(image_features, top_envs))}
    
    return res_ranks, frame_ranks, context_ranks, positive_ranks, env_ranks

def image_to_prompt(image, mode, clip_model_name, blip_model_name):
    if blip_model_name != ci.config.caption_model_name:
        ci.config.caption_model_name = blip_model_name
        ci.load_caption_model()

    if clip_model_name != ci.config.clip_model_name:
        ci.config.clip_model_name = clip_model_name
        ci.load_clip_model()

    image = image.convert('RGB')
    if mode == 'best':
        return ci.interrogate(image)
    elif mode == 'classic':
        return ci.interrogate_classic(image)
    elif mode == 'fast':
        return ci.interrogate_fast(image)
    elif mode == 'negative':
        return ci.interrogate_negative(image)

def prompt_tab():
    with gr.Column():
        with gr.Row():
            image = gr.Image(type = 'pil', label = "Image")
            with gr.Column():
                mode = gr.Radio(['best', 'fast', 'classic', 'negative'], label = 'Mode', value = 'best')
                clip_model = gr.Dropdown(list_clip_models(), value = ci.config.clip_model_name, label = 'CLIP Model')
                blip_model = gr.Dropdown(list_caption_models(), value = ci.config.caption_model_name, label = 'Caption Model')
        prompt = gr.Textbox(label = "Prompt")
    button = gr.Button("Generate prompt")
    button.click(image_to_prompt, inputs=[image, mode, clip_model, blip_model], outputs = prompt)

def analyze_tab():
    with gr.Column():
        with gr.Row():
            image = gr.Image(type = 'pil', label = "Image")
            model = gr.Dropdown(list_clip_models(), value = 'ViT-L-14/openai', label = 'CLIP Model')
        with gr.Row():
            res = gr.Label(label = "res", num_top_classes = 5)
            frame = gr.Label(label = "frame", num_top_classes = 5)        
            context = gr.Label(label = "context", num_top_classes = 5)
            aerialt = gr.Label(label = "aerial", num_top_classes = 5)
            positive = gr.Label(label = "positive", num_top_classes = 5)
            env = gr.Label(label = "env", num_top_classes = 5)
    button = gr.Button("Analyze")
    button.click(image_analysis, inputs = [image, model], outputs = [res, frame, context, aerial, positive, env])

with gr.Blocks() as ui:
    gr.Markdown("# <center>🕵️‍♂️ CLIP Interrogator 🕵️‍♂️</center>")
    with gr.Tab("Prompt"):
        prompt_tab()
    with gr.Tab("Analyze"):
        analyze_tab()

ui.launch(show_api = False, debug = True, share = args.share)
