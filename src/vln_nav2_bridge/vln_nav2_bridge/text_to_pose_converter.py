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
        shelf_pose = (-6.78, 10.96, 0.0)
        chair_pose = (-0.54, -0.69, 1.57)
        plant_pose = (-0.43, -2.92, 0.0)
        self._fallback_targets: Dict[str, Tuple[float, float, float]] = {
            "shelf area containing purple packages": shelf_pose,
            "right shelf with purple boxes": shelf_pose,
            "warehouse rack near the boxes": shelf_pose,
            "shelf area": shelf_pose,
            "warehouse rack": shelf_pose,
            "purple packages": shelf_pose,
            "purple package": shelf_pose,
            "package area": shelf_pose,
            "cart with boxes": shelf_pose,
            "a cart with boxes": shelf_pose,
            "purple boxes": shelf_pose,
            "purple box": shelf_pose,
            "right shelf": shelf_pose,
            "rack": shelf_pose,
            "shelf": shelf_pose,
            "boxes": shelf_pose,
            "plant": plant_pose,
            "black office chair": chair_pose,
            "office chair": chair_pose,
            "chair": chair_pose,
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

    def parse_model_json(self, text: str) -> Optional[Dict[str, object]]:
        candidates = re.findall(r"\{[^{}]*\}", text, flags=re.DOTALL)
        for candidate in candidates:
            try:
                data = json.loads(candidate)
            except json.JSONDecodeError:
                continue

            try:
                if "x" in data and "y" in data:
                    data["x"] = float(data["x"])
                    data["y"] = float(data["y"])
                    data["yaw"] = float(data.get("yaw", 0.0))
                if "confidence" in data:
                    data["confidence"] = float(data["confidence"])
                if "visible" in data and isinstance(data["visible"], str):
                    data["visible"] = data["visible"].strip().lower() in ("true", "yes", "1")
            except (TypeError, ValueError):
                continue

            if not any(key in data for key in ("target", "visible", "x", "y")):
                continue

            return data

        return None

    def _parse_json_pose(self, text: str) -> Optional[Tuple[float, float, float]]:
        data = self.parse_model_json(text)
        if data is None:
            return None
        if "x" not in data or "y" not in data:
            return None
        return (float(data["x"]), float(data["y"]), float(data.get("yaw", 0.0)))

    def resolve_named_target(
        self,
        instruction: str,
        model_target: str = "",
    ) -> Dict[str, object]:
        """Resolve a visually identified target name to the local semantic map."""
        model_target_text = model_target.lower().strip()
        instruction_text = instruction.lower().strip()

        parsed = self._resolve_named_target_from_text(model_target_text)
        if parsed is not None:
            return parsed

        if model_target_text and model_target_text not in ("unknown", "none", "not visible"):
            return {
                "ok": False,
                "reason": f"No semantic-map target matched model_target={model_target!r}.",
            }

        parsed = self._resolve_named_target_from_text(instruction_text)
        if parsed is not None:
            parsed["method"] = "visual_semantic_map_instruction_fallback"
            return parsed

        return {
            "ok": False,
            "reason": f"No semantic-map target matched model_target={model_target!r}.",
        }

    def _resolve_named_target_from_text(self, target_text: str) -> Optional[Dict[str, object]]:
        if not target_text:
            return None

        for key, pose in sorted(
            self._fallback_targets.items(),
            key=lambda item: len(item[0]),
            reverse=True,
        ):
            if key in target_text:
                x, y, yaw = pose
                if not self._is_safe(x, y):
                    return {
                        "ok": False,
                        "reason": f"Resolved target {key!r} is outside safe bounds.",
                    }
                return {
                    "ok": True,
                    "target": key,
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                    "method": "visual_semantic_map",
                }

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
