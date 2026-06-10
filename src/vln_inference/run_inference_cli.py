#!/usr/bin/env python3
import argparse
import os
import sys

from PIL import Image


def _patch_params4bit_constructor() -> None:
    """Compatibility patch for older bitsandbytes versions.

    Some transformer versions pass `_is_hf_initialized` into Params4bit.
    Older bitsandbytes releases don't accept that kwarg and crash during load.
    """
    try:
        from bitsandbytes.nn.modules import Params4bit
    except Exception:
        return

    original_new = Params4bit.__new__
    if getattr(original_new, "_vln_patched", False):
        return

    def _patched_new(cls, *args, **kwargs):
        kwargs.pop("_is_hf_initialized", None)
        return original_new(cls, *args, **kwargs)

    _patched_new._vln_patched = True
    Params4bit.__new__ = staticmethod(_patched_new)


def _build_model(model_path: str, enable_cpu_offload: bool):
    import torch
    from transformers import BitsAndBytesConfig, Qwen3VLForConditionalGeneration

    bnb_config = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_compute_dtype=torch.bfloat16,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_use_double_quant=True,
        llm_int8_enable_fp32_cpu_offload=enable_cpu_offload,
    )
    return Qwen3VLForConditionalGeneration.from_pretrained(
        model_path,
        quantization_config=bnb_config,
        device_map="auto",
        trust_remote_code=True,
    )


def _build_model_cpu(model_path: str):
    import torch
    from transformers import Qwen3VLForConditionalGeneration

    # Force full CPU inference to avoid GPU/offload dispatch issues.
    return Qwen3VLForConditionalGeneration.from_pretrained(
        model_path,
        device_map={"": "cpu"},
        torch_dtype=torch.float32,
        low_cpu_mem_usage=True,
        trust_remote_code=True,
    )


def _move_inputs_for_auto_device(inputs, model):
    """Avoid forcing CUDA when model uses device_map='auto' with CPU/disk offload."""
    try:
        model_device = getattr(model, "device", None)
        if model_device is not None and str(model_device) != "meta":
            return inputs.to(model_device)
    except Exception:
        pass
    # Keep on CPU; accelerate will dispatch as needed.
    return inputs


def _build_prompt(instruction: str) -> str:
    return (
        "You are a warehouse navigation assistant. "
        "Use the scene image to verify whether the target object is visible. "
        "Known map memory in map frame: "
        "shelf with purple boxes: x=-6.78, y=10.96, yaw=0.0; "
        "plant: x=-0.43, y=-2.92, yaw=0.0; "
        "chair: x=-0.54, y=-0.69, yaw=1.57. "
        "Return only one compact JSON object with keys "
        "target, visible, confidence, x, y, yaw. "
        "If the target is not visible but exists in map memory, use the memory coordinates "
        "and set visible=false. "
        f"User instruction: {instruction}"
    )


def _decode_generated_text(processor, inputs, generated_ids) -> str:
    generated_ids_trimmed = [
        out_ids[len(in_ids):] for in_ids, out_ids in zip(inputs.input_ids, generated_ids)
    ]
    output = processor.batch_decode(
        generated_ids_trimmed,
        skip_special_tokens=True,
        clean_up_tokenization_spaces=False,
    )
    return output[0].strip()


def run(
    model_path: str,
    image_path: str,
    instruction: str,
    max_new_tokens: int,
    force_cpu: bool,
    gpu_device: str,
) -> str:
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model path not found: {model_path}")
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Image path not found: {image_path}")

    if force_cpu:
        os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
    elif gpu_device:
        os.environ["CUDA_VISIBLE_DEVICES"] = gpu_device

    if force_cpu:
        model = _build_model_cpu(model_path=model_path)
    else:
        _patch_params4bit_constructor()
        model = _build_model(model_path=model_path, enable_cpu_offload=True)

    from transformers import AutoProcessor

    processor = AutoProcessor.from_pretrained(model_path, trust_remote_code=True)

    raw_image = Image.open(image_path).convert("RGB")
    prompt = _build_prompt(instruction)

    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": raw_image},
                {"type": "text", "text": prompt},
            ],
        }
    ]

    text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    inputs = processor(
        text=[text],
        images=[raw_image],
        padding=True,
        return_tensors="pt",
    )
    if force_cpu:
        inputs = inputs.to("cpu")
    else:
        inputs = _move_inputs_for_auto_device(inputs, model)

    try:
        generated_ids = model.generate(**inputs, max_new_tokens=max_new_tokens)
    except Exception as exc:
        if force_cpu or "meta tensors" not in str(exc):
            raise

        # Retry with a more conservative 4-bit setup when mixed offload dispatch is unstable.
        model = _build_model(model_path=model_path, enable_cpu_offload=False)
        inputs = _move_inputs_for_auto_device(inputs, model)
        generated_ids = model.generate(**inputs, max_new_tokens=max_new_tokens)

    return _decode_generated_text(processor, inputs, generated_ids)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--image-path", required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--max-new-tokens", type=int, default=256)
    parser.add_argument("--force-cpu", action="store_true")
    parser.add_argument("--gpu-device", default="0")
    args = parser.parse_args()

    try:
        result = run(
            model_path=args.model_path,
            image_path=args.image_path,
            instruction=args.instruction,
            max_new_tokens=args.max_new_tokens,
            force_cpu=args.force_cpu,
            gpu_device=args.gpu_device,
        )
        print(result)
        return 0
    except Exception as exc:
        print(f"INFERENCE_ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
