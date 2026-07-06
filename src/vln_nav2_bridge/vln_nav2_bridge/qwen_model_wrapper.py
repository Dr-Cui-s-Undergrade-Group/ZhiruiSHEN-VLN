import base64
import mimetypes
import os
import json
import select
import shutil
import subprocess
import threading
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Optional

_DEFAULT_QWEN_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
_DEFAULT_QWEN_MODEL = "qwen3-vl-flash"
_DOTENV_LOADED = False


def _load_dotenv_once() -> None:
    """Load a simple KEY=VALUE .env file without adding a runtime dependency."""
    global _DOTENV_LOADED
    if _DOTENV_LOADED:
        return
    _DOTENV_LOADED = True

    explicit_path = os.environ.get("VLN_DOTENV_PATH", "").strip()
    candidates = []
    if explicit_path:
        candidates.append(Path(explicit_path).expanduser())
    candidates.append(Path.cwd() / ".env")
    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / ".env")

    seen = set()
    for path in candidates:
        path = path.resolve()
        if path in seen:
            continue
        seen.add(path)
        if not path.exists():
            continue
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("export "):
                line = line[len("export "):].strip()
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key and key not in os.environ:
                os.environ[key] = value
        return


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _env_optional_int(name: str) -> Optional[int]:
    value = os.environ.get(name, "").strip()
    if not value:
        return None
    return int(value)


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    return float(value)


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
    """Qwen vision-language wrapper for API, local, and subprocess backends."""

    def __init__(
        self,
        model_path: str,
        max_new_tokens: int = 256,
        mode: str = "api",
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
        _load_dotenv_once()

        self.mode = str(mode).strip().lower()
        self.conda_env = conda_env
        self.inference_script_path = inference_script_path
        self.server_script_path = server_script_path
        self.python_executable = python_executable
        self.force_cpu = force_cpu
        self.gpu_device = gpu_device
        self.request_timeout_sec = request_timeout_sec
        self.api_model = (
            os.environ.get("QWEN_MODEL")
            or os.environ.get("DASHSCOPE_MODEL")
            or _DEFAULT_QWEN_MODEL
        ).strip()
        self.api_base_url = (
            os.environ.get("QWEN_BASE_URL")
            or os.environ.get("DASHSCOPE_BASE_URL")
            or _DEFAULT_QWEN_BASE_URL
        ).strip()
        self.api_key = (
            os.environ.get("QWEN_API_KEY")
            or os.environ.get("DASHSCOPE_API_KEY")
            or os.environ.get("ALIYUN_API_KEY")
            or ""
        ).strip()
        self.api_timeout_sec = _env_float("QWEN_API_TIMEOUT_SEC", request_timeout_sec)
        self.api_min_pixels = _env_optional_int("QWEN_API_MIN_PIXELS")
        self.api_max_pixels = _env_optional_int("QWEN_API_MAX_PIXELS")
        self.api_enable_thinking = _env_bool("QWEN_API_ENABLE_THINKING", False)
        self.api_json_mode = _env_bool("QWEN_API_JSON_MODE", True)
        self.api_high_resolution_images = _env_bool("QWEN_API_HIGH_RESOLUTION_IMAGES", False)
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
        if self.mode in ("api", "dashscope", "openai"):
            return self._infer_via_api(instruction=instruction, image_path=image_path)
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
        if self.mode in ("api", "dashscope", "openai"):
            self._ensure_api_configured()
            return
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
        if self.mode in ("api", "dashscope", "openai"):
            return bool(self.api_key and self.api_base_url and self.api_model)
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

    def _ensure_api_configured(self) -> None:
        if not self.api_key:
            raise RuntimeError(
                "API mode requires DASHSCOPE_API_KEY or QWEN_API_KEY. "
                "Set it in .env or export it before starting the node."
            )
        if not self.api_base_url:
            raise RuntimeError("API mode requires QWEN_BASE_URL or DASHSCOPE_BASE_URL.")
        if not self.api_model:
            raise RuntimeError("API mode requires QWEN_MODEL or DASHSCOPE_MODEL.")

    def _infer_via_api(self, instruction: str, image_path: str) -> str:
        self._ensure_api_configured()
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image path not found: {image_path}")

        image_part = {
            "type": "image_url",
            "image_url": {"url": self._image_data_url(image_path)},
        }
        if self.api_min_pixels is not None:
            image_part["min_pixels"] = self.api_min_pixels
        if self.api_max_pixels is not None:
            image_part["max_pixels"] = self.api_max_pixels

        payload = {
            "model": self.api_model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        image_part,
                        {"type": "text", "text": self._build_prompt(instruction)},
                    ],
                }
            ],
            "temperature": 0,
            "max_tokens": self.max_new_tokens,
            "stream": False,
            "enable_thinking": self.api_enable_thinking,
            "vl_high_resolution_images": self.api_high_resolution_images,
        }
        if self.api_json_mode:
            payload["response_format"] = {"type": "json_object"}

        request = urllib.request.Request(
            self._api_chat_url(),
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=self.api_timeout_sec) as response:
                body = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                f"Qwen API request failed: HTTP {exc.code}: {self._api_error_message(body)}"
            ) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"Qwen API request failed: {exc}") from exc

        try:
            result = json.loads(body)
            choices = result.get("choices") or []
            message = choices[0].get("message") if choices else {}
            content = message.get("content", "")
        except (AttributeError, IndexError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"Qwen API returned an unexpected response: {body[:500]}") from exc

        if isinstance(content, list):
            content = "".join(
                str(item.get("text", "")) if isinstance(item, dict) else str(item)
                for item in content
            )
        output = str(content).strip()
        if not output:
            raise RuntimeError(f"Qwen API returned empty output: {body[:500]}")
        return output

    def _api_chat_url(self) -> str:
        base_url = self.api_base_url.rstrip("/")
        if base_url.endswith("/chat/completions"):
            return base_url
        return f"{base_url}/chat/completions"

    @staticmethod
    def _image_data_url(image_path: str) -> str:
        path = Path(image_path)
        mime_type, _ = mimetypes.guess_type(str(path))
        if not mime_type or not mime_type.startswith("image/"):
            mime_type = "image/png"
        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
        return f"data:{mime_type};base64,{encoded}"

    @staticmethod
    def _api_error_message(body: str) -> str:
        try:
            data = json.loads(body)
        except json.JSONDecodeError:
            return body[:500]
        error = data.get("error") if isinstance(data, dict) else None
        if isinstance(error, dict):
            message = error.get("message") or error.get("code") or str(error)
            return str(message)[:500]
        return str(data)[:500]

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
