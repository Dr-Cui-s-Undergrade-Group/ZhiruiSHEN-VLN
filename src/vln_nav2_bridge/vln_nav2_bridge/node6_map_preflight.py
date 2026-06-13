#!/usr/bin/env python3
import argparse
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from PIL import Image


DEFAULT_MAP_YAML = (
    "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/warehouse_map.yaml"
)

DEFAULT_TARGETS: Dict[str, Tuple[float, float, float]] = {
    "plant": (-0.43, -2.92, 0.0),
    "chair": (-0.54, -0.69, 1.57),
    "shelf/package_area": (-6.78, 10.96, 0.0),
}


@dataclass(frozen=True)
class MapMetadata:
    image_path: str
    resolution: float
    origin_x: float
    origin_y: float
    occupied_thresh: float
    free_thresh: float
    negate: int


@dataclass(frozen=True)
class PoseCheck:
    name: str
    x: float
    y: float
    yaw: float
    col: int
    row: int
    center_state: str
    free_ratio: float
    occupied_ratio: float
    unknown_ratio: float
    sampled_cells: int

    def ok(self, min_free_ratio: float) -> bool:
        return self.center_state == "free" and self.free_ratio >= min_free_ratio


class SimpleOccupancyMap:
    def __init__(self, metadata: MapMetadata) -> None:
        self.metadata = metadata
        self.image = Image.open(metadata.image_path).convert("RGBA")
        self.width, self.height = self.image.size

    @classmethod
    def from_yaml(cls, map_yaml: str) -> "SimpleOccupancyMap":
        metadata = _load_map_metadata(map_yaml)
        return cls(metadata)

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        col = int(round((x - self.metadata.origin_x) / self.metadata.resolution))
        row = int(
            round(
                (self.height - 1)
                - ((y - self.metadata.origin_y) / self.metadata.resolution)
            )
        )
        return col, row

    def pixel_to_world(self, col: int, row: int) -> Tuple[float, float]:
        x = self.metadata.origin_x + (col * self.metadata.resolution)
        y = self.metadata.origin_y + ((self.height - 1 - row) * self.metadata.resolution)
        return x, y

    def check_pose(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float,
        radius_m: float,
    ) -> PoseCheck:
        col, row = self.world_to_pixel(x, y)
        radius_px = max(0, int(round(radius_m / self.metadata.resolution)))
        free = 0
        occupied = 0
        unknown = 0
        sampled = 0

        if not self._in_bounds(col, row):
            return PoseCheck(
                name=name,
                x=x,
                y=y,
                yaw=yaw,
                col=col,
                row=row,
                center_state="outside_map",
                free_ratio=0.0,
                occupied_ratio=0.0,
                unknown_ratio=1.0,
                sampled_cells=0,
            )

        for sample_row in range(
            max(0, row - radius_px),
            min(self.height, row + radius_px + 1),
        ):
            for sample_col in range(
                max(0, col - radius_px),
                min(self.width, col + radius_px + 1),
            ):
                if radius_px > 0:
                    dist_sq = (sample_row - row) ** 2 + (sample_col - col) ** 2
                    if dist_sq > radius_px**2:
                        continue
                sampled += 1
                state = self._cell_state(sample_col, sample_row)
                if state == "free":
                    free += 1
                elif state == "occupied":
                    occupied += 1
                else:
                    unknown += 1

        sampled = max(1, sampled)
        return PoseCheck(
            name=name,
            x=x,
            y=y,
            yaw=yaw,
            col=col,
            row=row,
            center_state=self._cell_state(col, row),
            free_ratio=free / sampled,
            occupied_ratio=occupied / sampled,
            unknown_ratio=unknown / sampled,
            sampled_cells=sampled,
        )

    def find_nearest_free_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        radius_m: float,
        min_free_ratio: float,
        max_search_m: float,
    ) -> Optional[PoseCheck]:
        step_m = max(self.metadata.resolution, 0.05)
        ring_count = int(math.ceil(max_search_m / step_m))
        for ring in range(ring_count + 1):
            search_radius = ring * step_m
            if search_radius <= 0.0:
                candidates = [(x, y)]
            else:
                samples = max(16, int(math.ceil((2.0 * math.pi * search_radius) / step_m)))
                candidates = [
                    (
                        x + search_radius * math.cos((2.0 * math.pi * idx) / samples),
                        y + search_radius * math.sin((2.0 * math.pi * idx) / samples),
                    )
                    for idx in range(samples)
                ]

            best_in_ring: Optional[PoseCheck] = None
            for cand_x, cand_y in candidates:
                check = self.check_pose(
                    name="nearest_free",
                    x=cand_x,
                    y=cand_y,
                    yaw=yaw,
                    radius_m=radius_m,
                )
                if check.ok(min_free_ratio):
                    if best_in_ring is None:
                        best_in_ring = check
                    else:
                        old_dist = math.hypot(best_in_ring.x - x, best_in_ring.y - y)
                        new_dist = math.hypot(check.x - x, check.y - y)
                        if new_dist < old_dist:
                            best_in_ring = check
            if best_in_ring is not None:
                return best_in_ring

        return None

    def _cell_state(self, col: int, row: int) -> str:
        if not self._in_bounds(col, row):
            return "outside_map"

        red, green, blue, alpha = self.image.getpixel((col, row))
        if alpha == 0:
            return "unknown"

        brightness = (float(red) + float(green) + float(blue)) / (3.0 * 255.0)
        if self.metadata.negate:
            occupancy = brightness
        else:
            occupancy = 1.0 - brightness

        if occupancy > self.metadata.occupied_thresh:
            return "occupied"
        if occupancy < self.metadata.free_thresh:
            return "free"
        return "unknown"

    def _in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height


def _load_map_metadata(map_yaml: str) -> MapMetadata:
    values = _parse_simple_yaml(map_yaml)
    map_dir = os.path.dirname(os.path.abspath(map_yaml))
    image_value = values["image"].strip("\"'")
    image_path = image_value
    if not os.path.isabs(image_path):
        image_path = os.path.join(map_dir, image_path)

    origin = _parse_float_list(values["origin"])
    if len(origin) < 2:
        raise ValueError(f"Map origin must contain at least x,y: {values['origin']!r}")

    return MapMetadata(
        image_path=image_path,
        resolution=float(values["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        occupied_thresh=float(values.get("occupied_thresh", 0.65)),
        free_thresh=float(values.get("free_thresh", 0.196)),
        negate=int(values.get("negate", 0)),
    )


def _parse_simple_yaml(path: str) -> Dict[str, str]:
    values: Dict[str, str] = {}
    with open(path, "r", encoding="utf-8") as file_obj:
        for raw_line in file_obj:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    return values


def _parse_float_list(text: str) -> List[float]:
    stripped = text.strip().strip("[]")
    if not stripped:
        return []
    return [float(item.strip()) for item in stripped.split(",")]


def _parse_named_point(text: str) -> Tuple[str, Tuple[float, float, float]]:
    if ":" not in text:
        raise argparse.ArgumentTypeError("Point must be NAME:X,Y[,YAW].")

    name, value = text.split(":", 1)
    parts = [item.strip() for item in value.split(",")]
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError("Point must be NAME:X,Y[,YAW].")

    yaw = float(parts[2]) if len(parts) == 3 else 0.0
    return name.strip(), (float(parts[0]), float(parts[1]), yaw)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wait_for_odom(topic: str, timeout_sec: float) -> Optional[Tuple[float, float, float]]:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class OdomCapture(Node):
        def __init__(self, odom_topic: str) -> None:
            super().__init__("node6_map_preflight")
            self.latest: Optional[Tuple[float, float, float]] = None
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        def _on_odom(self, msg: Odometry) -> None:
            pose = msg.pose.pose
            yaw = _yaw_from_quaternion(
                x=float(pose.orientation.x),
                y=float(pose.orientation.y),
                z=float(pose.orientation.z),
                w=float(pose.orientation.w),
            )
            self.latest = (
                float(pose.position.x),
                float(pose.position.y),
                yaw,
            )

    rclpy.init(args=None)
    node = OdomCapture(topic)
    try:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest is not None:
                return node.latest
        return None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _format_check(
    check: PoseCheck,
    occupancy_map: SimpleOccupancyMap,
    min_free_ratio: float,
    radius_m: float,
    nearest_free_search_m: float,
) -> List[str]:
    status = "OK" if check.ok(min_free_ratio) else "BLOCKED"
    lines = [
        (
            f"[{status}] {check.name}: "
            f"world=({check.x:.3f}, {check.y:.3f}, yaw={check.yaw:.3f}), "
            f"pixel=({check.col}, {check.row}), center={check.center_state}, "
            f"free={check.free_ratio:.1%}, occupied={check.occupied_ratio:.1%}, "
            f"unknown={check.unknown_ratio:.1%}, samples={check.sampled_cells}"
        )
    ]
    if check.ok(min_free_ratio):
        return lines

    nearest = occupancy_map.find_nearest_free_pose(
        x=check.x,
        y=check.y,
        yaw=check.yaw,
        radius_m=radius_m,
        min_free_ratio=min_free_ratio,
        max_search_m=nearest_free_search_m,
    )
    if nearest is None:
        lines.append(
            f"        nearest_free: none within {nearest_free_search_m:.2f} m"
        )
    else:
        distance = math.hypot(nearest.x - check.x, nearest.y - check.y)
        lines.append(
            "        nearest_free: "
            f"world=({nearest.x:.3f}, {nearest.y:.3f}, yaw={nearest.yaw:.3f}), "
            f"distance={distance:.3f} m, free={nearest.free_ratio:.1%}"
        )
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Check Node 6 robot/target poses against the static occupancy map before "
            "running Nav2 trials."
        )
    )
    parser.add_argument("--map-yaml", default=DEFAULT_MAP_YAML)
    parser.add_argument("--odom-topic", default="/chassis/odom")
    parser.add_argument("--odom-timeout-sec", type=float, default=5.0)
    parser.add_argument("--skip-odom", action="store_true")
    parser.add_argument("--robot-radius-m", type=float, default=0.30)
    parser.add_argument("--min-free-ratio", type=float, default=0.95)
    parser.add_argument("--nearest-free-search-m", type=float, default=2.0)
    parser.add_argument(
        "--no-default-targets",
        action="store_true",
        help="Do not check built-in plant/chair/shelf semantic targets.",
    )
    parser.add_argument(
        "--point",
        action="append",
        default=[],
        type=_parse_named_point,
        help="Extra point to check, formatted as NAME:X,Y[,YAW].",
    )
    args = parser.parse_args()

    occupancy_map = SimpleOccupancyMap.from_yaml(args.map_yaml)
    print(
        "map: "
        f"{args.map_yaml} -> {occupancy_map.metadata.image_path}, "
        f"size={occupancy_map.width}x{occupancy_map.height}, "
        f"resolution={occupancy_map.metadata.resolution:.3f} m/px"
    )
    print(
        "thresholds: "
        f"free_ratio>={args.min_free_ratio:.2f}, "
        f"check_radius={args.robot_radius_m:.2f} m"
    )

    checks: List[Tuple[str, Tuple[float, float, float]]] = []
    if not args.skip_odom:
        odom = _wait_for_odom(args.odom_topic, args.odom_timeout_sec)
        if odom is None:
            print(
                f"[WARN] No odom sample received from {args.odom_topic!r} "
                f"within {args.odom_timeout_sec:.1f}s."
            )
        else:
            checks.append((f"current_odom:{args.odom_topic}", odom))

    if not args.no_default_targets:
        checks.extend(DEFAULT_TARGETS.items())
    checks.extend(args.point)

    blocked = 0
    for name, (x, y, yaw) in checks:
        check = occupancy_map.check_pose(
            name=name,
            x=x,
            y=y,
            yaw=yaw,
            radius_m=args.robot_radius_m,
        )
        if not check.ok(args.min_free_ratio):
            blocked += 1
        for line in _format_check(
            check=check,
            occupancy_map=occupancy_map,
            min_free_ratio=args.min_free_ratio,
            radius_m=args.robot_radius_m,
            nearest_free_search_m=args.nearest_free_search_m,
        ):
            print(line)

    if blocked:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
