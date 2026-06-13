import os
import json
import select
import shutil
import subprocess
import threading
from collections import deque
from typing import Optional


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


def _move_inputs_for_auto_device(inputs, model):
    """Avoid forcing CUDA when model uses device_map='auto' with CPU/disk offload."""
    try:
        model_device = getattr(model, "device", None)
        if model_device is not None and str(model_device) != "meta":
            return inputs.to(model_device)
    except Exception:
        pass
    return inputs


class QwenVLWrapper:
    """Local Qwen3-VL wrapper with lazy loading for Node 5."""

    def __init__(
        self,
        model_path: str,
        max_new_tokens: int = 256,
        mode: str = "subprocess",
        conda_env: str = "isaaclab",
        inference_script_path: str = "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/src/vln_inference/run_inference_cli.py",
        server_script_path: str = "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/src/vln_inference/run_inference_server.py",
        python_executable: str = "/home/bluepoisons/miniconda3/envs/isaaclab/bin/python",
        force_cpu: bool = False,
        gpu_device: str = "0",
        request_timeout_sec: float = 180.0,
    ) -> None:
        self.model_path = model_path
        self.max_new_tokens = max_new_tokens
        self.mode = mode
        self.conda_env = conda_env
        self.inference_script_path = inference_script_path
        self.server_script_path = server_script_path
        self.python_executable = python_executable
        self.force_cpu = force_cpu
        self.gpu_device = gpu_device
        self.request_timeout_sec = request_timeout_sec
        self._model = None
        self._processor = None
        self._worker = None
        self._worker_lock = threading.Lock()
        self._worker_stderr = deque(maxlen=80)
        self._worker_stdout_noise = deque(maxlen=20)
        self._stderr_thread = None

    def _ensure_loaded(self) -> None:
        if self.mode != "local":
            return

        if self._model is not None and self._processor is not None:
            return

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model path not found: {self.model_path}")

        if self.force_cpu:
            os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
        elif self.gpu_device:
            os.environ["CUDA_VISIBLE_DEVICES"] = self.gpu_device

        # Import heavy ML dependencies only in local mode.
        from PIL import Image  # noqa: F401
        import torch
        from transformers import AutoProcessor, BitsAndBytesConfig, Qwen3VLForConditionalGeneration

        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_compute_dtype=torch.bfloat16,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_use_double_quant=True,
            llm_int8_enable_fp32_cpu_offload=True,
        )

        _patch_params4bit_constructor()

        self._model = Qwen3VLForConditionalGeneration.from_pretrained(
            self.model_path,
            quantization_config=bnb_config,
            device_map="auto",
            trust_remote_code=True,
        )
        self._processor = AutoProcessor.from_pretrained(
            self.model_path,
            trust_remote_code=True,
        )

    def infer_goal_text(self, instruction: str, image_path: str) -> str:
        """Run vision-language inference and return model text output."""
        if self.mode == "subprocess":
            return self._infer_via_subprocess(instruction=instruction, image_path=image_path)
        if self.mode in ("server", "persistent_subprocess"):
            return self._infer_via_worker(instruction=instruction, image_path=image_path)

        self._ensure_loaded()

        from PIL import Image

        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image path not found: {image_path}")

        raw_image = Image.open(image_path).convert("RGB")

        prompt = self._build_prompt(instruction)

        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": raw_image},
                    {"type": "text", "text": prompt},
                ],
            }
        ]

        text = self._processor.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True,
        )
        inputs = self._processor(
            text=[text],
            images=[raw_image],
            padding=True,
            return_tensors="pt",
        )
        inputs = _move_inputs_for_auto_device(inputs, self._model)

        generated_ids = self._model.generate(
            **inputs,
            max_new_tokens=self.max_new_tokens,
        )
        return self._decode_generated_text(inputs=inputs, generated_ids=generated_ids)

    def start(self, timeout_sec: Optional[float] = None) -> None:
        """Start and health-check the configured inference backend."""
        if self.mode in ("server", "persistent_subprocess"):
            self._request_worker(
                {"type": "ping"},
                timeout_sec=timeout_sec or self.request_timeout_sec,
            )
            return
        if self.mode == "local":
            self._ensure_loaded()

    def shutdown(self) -> None:
        """Best-effort shutdown for the persistent worker process."""
        with self._worker_lock:
            proc = self._worker
            self._worker = None

        if proc is None:
            return
        if proc.poll() is not None:
            return

        try:
            if proc.stdin:
                proc.stdin.write(json.dumps({"type": "shutdown"}) + "\n")
                proc.stdin.flush()
        except Exception:
            pass

        try:
            proc.terminate()
            proc.wait(timeout=5)
        except Exception:
            proc.kill()

    def is_ready(self) -> bool:
        if self.mode in ("server", "persistent_subprocess"):
            return self._worker is not None and self._worker.poll() is None
        if self.mode == "local":
            return self._model is not None and self._processor is not None
        return True

    @staticmethod
    def _build_prompt(instruction: str) -> str:
        return (
            "You are a robot camera target verifier in a warehouse. "
            "Look only at the current camera image. Do not use map memory or prior coordinates. "
            "Decide whether the object requested by the user is visibly present in this image. "
            "Return only one compact JSON object with keys "
            "target, visible, confidence, horizontal_position, evidence. "
            "horizontal_position must be left, center, right, or unknown. "
            "If the requested object is not visible, set visible=false, confidence=0.0, "
            "horizontal_position=unknown, and do not invent coordinates. "
            f"User instruction: {instruction}"
        )

    def _decode_generated_text(self, inputs, generated_ids) -> str:
        generated_ids_trimmed = [
            out_ids[len(in_ids):] for in_ids, out_ids in zip(inputs.input_ids, generated_ids)
        ]
        output = self._processor.batch_decode(
            generated_ids_trimmed,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )
        return output[0].strip()

    def _infer_via_subprocess(self, instruction: str, image_path: str) -> str:
        if not os.path.exists(self.inference_script_path):
            raise FileNotFoundError(
                f"Inference script not found: {self.inference_script_path}"
            )

        cmd = [
            "conda",
            "run",
            "-n",
            self.conda_env,
            "python",
            self.inference_script_path,
            "--model-path",
            self.model_path,
            "--image-path",
            image_path,
            "--instruction",
            instruction,
            "--max-new-tokens",
            str(self.max_new_tokens),
        ]

        env = os.environ.copy()
        if self.force_cpu:
            cmd.append("--force-cpu")
            env["CUDA_VISIBLE_DEVICES"] = "-1"
        elif self.gpu_device:
            cmd.extend(["--gpu-device", self.gpu_device])
            env["CUDA_VISIBLE_DEVICES"] = self.gpu_device
        proc = subprocess.run(cmd, check=False, capture_output=True, text=True, env=env)
        if proc.returncode != 0:
            stderr = proc.stderr.strip() if proc.stderr else "No stderr"
            raise RuntimeError(f"Subprocess inference failed: {stderr}")

        output = proc.stdout.strip()
        if not output:
            raise RuntimeError("Subprocess inference returned empty output.")
        return output

    def _infer_via_worker(self, instruction: str, image_path: str) -> str:
        response = self._request_worker(
            {
                "type": "predict",
                "instruction": instruction,
                "image_path": image_path,
                "max_new_tokens": self.max_new_tokens,
            },
            timeout_sec=self.request_timeout_sec,
        )
        output = str(response.get("output", "")).strip()
        if not output:
            raise RuntimeError("Persistent inference worker returned empty output.")
        return output

    def _request_worker(self, payload: dict, timeout_sec: float) -> dict:
        with self._worker_lock:
            self._ensure_worker_started()
            proc = self._worker
            if proc is None or proc.stdin is None or proc.stdout is None:
                raise RuntimeError("Persistent inference worker is not available.")

            if proc.poll() is not None:
                raise RuntimeError(self._format_worker_failure("worker exited before request"))

            try:
                proc.stdin.write(json.dumps(payload, ensure_ascii=False) + "\n")
                proc.stdin.flush()
            except OSError as exc:
                raise RuntimeError(
                    self._format_worker_failure(f"failed to write worker request: {exc}")
                ) from exc

            timeout_at = _monotonic_seconds() + max(0.0, timeout_sec)
            while _monotonic_seconds() < timeout_at:
                if proc.poll() is not None:
                    raise RuntimeError(
                        self._format_worker_failure("worker exited while waiting for response")
                    )

                remaining = max(0.0, timeout_at - _monotonic_seconds())
                readable, _, _ = select.select([proc.stdout], [], [], min(0.25, remaining))
                if not readable:
                    continue

                line = proc.stdout.readline()
                if not line:
                    continue
                line = line.strip()
                if not line:
                    continue

                try:
                    response = json.loads(line)
                except json.JSONDecodeError:
                    self._worker_stdout_noise.append(line)
                    continue

                if not response.get("ok"):
                    error = response.get("error", "unknown worker error")
                    raise RuntimeError(self._format_worker_failure(str(error)))
                return response

            raise TimeoutError(
                self._format_worker_failure(
                    f"worker request timed out after {timeout_sec:.1f}s"
                )
            )

    def _ensure_worker_started(self) -> None:
        if self._worker is not None and self._worker.poll() is None:
            return

        if not os.path.exists(self.server_script_path):
            raise FileNotFoundError(f"Inference server script not found: {self.server_script_path}")
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model path not found: {self.model_path}")

        cmd = self._build_worker_cmd()
        env = os.environ.copy()
        if self.force_cpu:
            env["CUDA_VISIBLE_DEVICES"] = "-1"
        elif self.gpu_device:
            env["CUDA_VISIBLE_DEVICES"] = self.gpu_device

        self._worker_stderr.clear()
        self._worker_stdout_noise.clear()
        self._worker = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=env,
        )
        self._stderr_thread = threading.Thread(
            target=self._drain_worker_stderr,
            daemon=True,
        )
        self._stderr_thread.start()

    def _build_worker_cmd(self) -> list:
        python_executable = self.python_executable or os.environ.get("VLN_MODEL_PYTHON", "")
        if python_executable and os.path.exists(python_executable):
            base_cmd = [python_executable, "-u", self.server_script_path]
        else:
            conda = shutil.which("conda") or "conda"
            base_cmd = [
                conda,
                "run",
                "--no-capture-output",
                "-n",
                self.conda_env,
                "python",
                "-u",
                self.server_script_path,
            ]

        cmd = [
            *base_cmd,
            "--model-path",
            self.model_path,
            "--max-new-tokens",
            str(self.max_new_tokens),
        ]
        if self.force_cpu:
            cmd.append("--force-cpu")
        elif self.gpu_device:
            cmd.extend(["--gpu-device", self.gpu_device])
        return cmd

    def _drain_worker_stderr(self) -> None:
        proc = self._worker
        if proc is None or proc.stderr is None:
            return
        for line in proc.stderr:
            text = line.rstrip()
            if text:
                self._worker_stderr.append(text)

    def _format_worker_failure(self, message: str) -> str:
        details = [message]
        if self._worker_stdout_noise:
            details.append("stdout_noise=" + " | ".join(self._worker_stdout_noise))
        if self._worker_stderr:
            details.append("stderr_tail=" + " | ".join(self._worker_stderr))
        return "; ".join(details)


def _monotonic_seconds() -> float:
    import time

    return time.monotonic()
