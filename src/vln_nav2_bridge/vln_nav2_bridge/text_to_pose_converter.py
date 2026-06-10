import json
import re
from typing import Dict, Optional, Tuple


class TextToPoseConverter:
    """Convert model output text into safe map-frame target coordinates."""

    def __init__(self, min_x: float = -8.0, max_x: float = 10.0, min_y: float = -12.0, max_y: float = 15.0) -> None:
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self._fallback_targets: Dict[str, Tuple[float, float, float]] = {
            "purple box": (-6.78, 10.96, 0.0),
            "purple boxes": (-6.78, 10.96, 0.0),
            "right shelf": (-6.78, 10.96, 0.0),
            "shelf": (-6.78, 10.96, 0.0),
            "plant": (-0.43, -2.92, 0.0),
            "chair": (-0.54, -0.69, 1.57),
            "office chair": (-0.54, -0.69, 1.57),
            "robot": (0.0, 0.0, 0.0),
            "center": (0.0, 0.0, 0.0),
        }

    def convert(self, instruction: str, model_output: str) -> Dict[str, object]:
        """Return dict with x, y, yaw and conversion metadata."""
        parsed = self._parse_json_pose(model_output)
        method = "json"

        if parsed is None:
            parsed = self._fallback_from_instruction(instruction)
            method = "instruction_fallback"

        if parsed is None:
            parsed = self._fallback_from_text(instruction, model_output)
            method = "fallback"

        if parsed is None:
            return {
                "ok": False,
                "reason": "No pose found in model output and fallback rules did not match.",
            }

        x, y, yaw = parsed
        if not self._is_safe(x, y):
            return {
                "ok": False,
                "reason": (
                    f"Target out of safe range: x={x:.2f}, y={y:.2f}, "
                    f"allowed_x=[{self.min_x}, {self.max_x}], allowed_y=[{self.min_y}, {self.max_y}]"
                ),
                "method": method,
            }

        return {
            "ok": True,
            "x": x,
            "y": y,
            "yaw": yaw,
            "method": method,
        }

    def _parse_json_pose(self, text: str) -> Optional[Tuple[float, float, float]]:
        candidates = re.findall(r"\{[^{}]*\}", text, flags=re.DOTALL)
        for candidate in candidates:
            try:
                data = json.loads(candidate)
            except json.JSONDecodeError:
                continue

            if "x" not in data or "y" not in data:
                continue

            try:
                x = float(data["x"])
                y = float(data["y"])
                yaw = float(data.get("yaw", 0.0))
            except (TypeError, ValueError):
                continue

            return (x, y, yaw)

        return None

    def _fallback_from_text(self, instruction: str, model_output: str) -> Optional[Tuple[float, float, float]]:
        combined = f"{instruction}\n{model_output}".lower()
        for key, pose in self._fallback_targets.items():
            if key in combined:
                return pose
        return None

    def _fallback_from_instruction(self, instruction: str) -> Optional[Tuple[float, float, float]]:
        instruction_lower = instruction.lower()
        for key, pose in self._fallback_targets.items():
            if key in instruction_lower:
                return pose
        return None

    def _is_safe(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y
