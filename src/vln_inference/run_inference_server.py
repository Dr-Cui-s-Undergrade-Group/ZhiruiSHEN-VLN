#!/usr/bin/env python3
"""Persistent Qwen3-VL inference worker for the ROS bridge.

Protocol:
  stdin: one JSON object per line
  stdout: one JSON object per line

The worker loads the model once at startup, then serves repeated inference
requests without paying model-load cost for every VLN instruction.
"""
import argparse
import json
import os
import sys
import traceback

from PIL import Image

from run_inference_cli import (
    _build_model,
    _build_model_cpu,
    _build_prompt,
    _decode_generated_text,
    _move_inputs_for_auto_device,
    _patch_params4bit_constructor,
)


def _write_response(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def _load_model(model_path: str, force_cpu: bool, gpu_device: str):
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
    return model, processor


def _predict(
    model,
    processor,
    image_path: str,
    instruction: str,
    max_new_tokens: int,
    force_cpu: bool,
) -> str:
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Image path not found: {image_path}")

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

    text = processor.apply_chat_template(
        messages,
        tokenize=False,
        add_generation_prompt=True,
    )
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

    generated_ids = model.generate(**inputs, max_new_tokens=max_new_tokens)
    return _decode_generated_text(processor, inputs, generated_ids)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--max-new-tokens", type=int, default=256)
    parser.add_argument("--force-cpu", action="store_true")
    parser.add_argument("--gpu-device", default="0")
    args = parser.parse_args()

    try:
        if not os.path.exists(args.model_path):
            raise FileNotFoundError(f"Model path not found: {args.model_path}")
        print(
            f"Loading Qwen3-VL worker: model={args.model_path}, "
            f"force_cpu={args.force_cpu}, gpu_device={args.gpu_device}",
            file=sys.stderr,
            flush=True,
        )
        model, processor = _load_model(
            model_path=args.model_path,
            force_cpu=args.force_cpu,
            gpu_device=args.gpu_device,
        )
        print("Qwen3-VL worker ready.", file=sys.stderr, flush=True)
    except Exception as exc:
        traceback.print_exc(file=sys.stderr)
        _write_response(
            {
                "ok": False,
                "type": "startup",
                "error": f"{type(exc).__name__}: {exc}",
            }
        )
        return 1

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        try:
            request = json.loads(line)
            request_type = str(request.get("type", "predict"))
            if request_type == "ping":
                _write_response({"ok": True, "type": "pong", "model_loaded": True})
                continue
            if request_type == "shutdown":
                _write_response({"ok": True, "type": "shutdown"})
                return 0
            if request_type != "predict":
                raise ValueError(f"Unsupported request type: {request_type}")

            result = _predict(
                model=model,
                processor=processor,
                image_path=str(request["image_path"]),
                instruction=str(request["instruction"]),
                max_new_tokens=int(request.get("max_new_tokens", args.max_new_tokens)),
                force_cpu=args.force_cpu,
            )
            _write_response({"ok": True, "type": "predict", "output": result})
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            _write_response(
                {
                    "ok": False,
                    "type": "error",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
