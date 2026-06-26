#!/usr/bin/env python3
"""Phase 1 diagnostic: scan-vs-map residual along a teleop path (odom-truth).

Goal: quantify how well the rolling /scan matches the static warehouse map at
the *true* robot pose (raw /chassis/odom, NOT AMCL). If residual is high in the
shelf/corridor segment but low near plant/chair, the culprit is the static
occupancy map (expressed too coarsely for shelf/forklift/purple-box geometry),
not the AMCL algorithm. This node never touches AMCL and never feeds Nav2.

Per published /scan, at the current odom-truth pose:
  - raymarch each finite scan beam through the static occupancy grid to the
    first occupied cell (map-predicted hit distance);
  - compare to the scan-measured range -> per-beam residual = |meas - pred|;
  - aggregate mean / median / p90 residual, finite-beam ratio, beam coverage.

A row is written to CSV when the robot has travelled >= sample_step_m
(default 0.5 m) or >= min_dt_sec (default 0.3 s) since the last row, so the
output is a path-residual trace usable directly for a heatmap/curve.

Run under odom-truth baseline (amcl.tf_broadcast=false + static map->odom),
teleop slowly along origin -> purple boxes, then inspect the CSV.
"""
from __future__ import annotations

import math
import os
import statistics
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from PIL import Image


DEFAULT_MAP_YAML = (
    "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/data/warehouse_map.yaml"
)
DEFAULT_OUTPUT = (
    "/home/bluepoisons/Desktop/FURP/VLN/ZhiruiSHEN-VLN/"
    "data/node8_scan_map_residual_diagnostic.csv"
)

CSV_HEADER = [
    "timestamp",
    "odom_x",
    "odom_y",
    "odom_yaw",
    "distance_from_start_m",
    "beams_total",
    "beams_finite",
    "finite_ratio",
    "residual_mean_m",
    "residual_median_m",
    "residual_p90_m",
    "map_hit_ratio",
    "map_unknown_off_map_ratio",
    "mean_measured_range_m",
    "mean_predicted_range_m",
]


@dataclass(frozen=True)
class MapMetadata:
    image_path: str
    resolution: float
    origin_x: float
    origin_y: float
    negate: int
    occupied_thresh: float
    free_thresh: float


class OccupancyMap:
    """Static occupancy grid reader (mirrors node6_map_preflight conventions)."""

    def __init__(self, metadata: MapMetadata) -> None:
        self.metadata = metadata
        # Decode once into a per-cell occupancy probability array for fast lookup.
        image = Image.open(metadata.image_path).convert("RGBA")
        self.width, self.height = image.size
        rgba = np.asarray(image, dtype=np.float32)
        alpha = rgba[:, :, 3]
        brightness = rgba[:, :, :3].sum(axis=2) / (3.0 * 255.0)
        occupancy = brightness if metadata.negate else (1.0 - brightness)
        # Build a flat lookup: 0 unknown, 1 free, 2 occupied. Row 0 is top.
        state = np.zeros((self.height, self.width), dtype=np.uint8)
        state[(alpha == 0)] = 0  # unknown
        free_mask = (alpha != 0) & (occupancy < metadata.free_thresh)
        occ_mask = (alpha != 0) & (occupancy > metadata.occupied_thresh)
        state[free_mask] = 1
        state[occ_mask] = 2
        # Flip vertically so that index [0,:] is the bottom row (y minimal),
        # matching world_to_pixel row convention used elsewhere.
        self.state = np.flipud(state)

    @classmethod
    def from_yaml(cls, map_yaml: str) -> "OccupancyMap":
        meta = _load_map_metadata(map_yaml)
        return cls(meta)

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        col = int(round((x - self.metadata.origin_x) / self.metadata.resolution))
        # After flipud, row index 0 is the min-y row, growing upward.
        row = int(round((y - self.metadata.origin_y) / self.metadata.resolution))
        return col, row

    def cell_state(self, col: int, row: int) -> int:
        """0 unknown, 1 free, 2 occupied, -1 outside map."""
        if not (0 <= col < self.width and 0 <= row < self.height):
            return -1
        return int(self.state[row, col])


def _load_map_metadata(map_yaml: str) -> MapMetadata:
    values = _parse_simple_yaml(map_yaml)
    yaml_dir = os.path.dirname(os.path.abspath(map_yaml))
    image_path = values["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(yaml_dir, image_path)
    return MapMetadata(
        image_path=image_path,
        resolution=float(values["resolution"]),
        origin_x=float(_parse_float_list(values["origin"])[0]),
        origin_y=float(_parse_float_list(values["origin"])[1]),
        negate=int(values.get("negate", 0)),
        occupied_thresh=float(values.get("occupied_thresh", 0.65)),
        free_thresh=float(values.get("free_thresh", 0.196)),
    )


def _parse_simple_yaml(path: str) -> Dict[str, str]:
    values: Dict[str, str] = {}
    with open(path, "r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    return values


def _parse_float_list(text: str) -> List[float]:
    stripped = text.strip().strip("[]")
    return [float(item.strip()) for item in stripped.split(",") if item.strip()]


def _stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _percentile(values: List[float], pct: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (pct / 100.0) * (len(ordered) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return ordered[lo]
    frac = rank - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _raycast_predicted_range(
    occ: OccupancyMap,
    origin_x: float,
    origin_y: float,
    yaw_world: float,
    beam_angle_world: float,
    range_max: float,
) -> Tuple[float, int]:
    """March a single beam from the true pose; return (predicted_hit_range, end_state).

    end_state: 2 occupied hit, 0 unknown/traversed-only, -1 off-map.
    predicted_hit_range is range_max if no occupied cell is found.
    """
    res = occ.metadata.resolution
    start_col, start_row = occ.world_to_pixel(origin_x, origin_y)
    direction = beam_angle_world  # absolute angle in map/world frame
    step = max(res, 0.02)
    n_steps = int(math.ceil(range_max / step))
    predicted = range_max
    end_state = 0
    for i in range(1, n_steps + 1):
        dist = i * step
        wx = origin_x + dist * math.cos(direction)
        wy = origin_y + dist * math.sin(direction)
        col, row = occ.world_to_pixel(wx, wy)
        state = occ.cell_state(col, row)
        if state == -1:
            end_state = -1
            break
        if state == 2:  # occupied -> predicted hit
            predicted = dist
            end_state = 2
            break
    return predicted, end_state


class ScanMapResidualNode(Node):
    def __init__(self) -> None:
        super().__init__("node8_scan_map_residual")

        self.declare_parameter("map_yaml", DEFAULT_MAP_YAML)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("output_csv", DEFAULT_OUTPUT)
        self.declare_parameter("sample_step_m", 0.5)
        self.declare_parameter("min_dt_sec", 0.3)
        self.declare_parameter("beam_stride", 1)
        self.declare_parameter("residual_clip_m", 5.0)

        map_yaml = str(self.get_parameter("map_yaml").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.output_csv = str(self.get_parameter("output_csv").value)
        self.sample_step_m = float(self.get_parameter("sample_step_m").value)
        self.min_dt_sec = float(self.get_parameter("min_dt_sec").value)
        self.beam_stride = max(1, int(self.get_parameter("beam_stride").value))
        self.residual_clip_m = float(self.get_parameter("residual_clip_m").value)

        self.occ = OccupancyMap.from_yaml(map_yaml)
        self.get_logger().info(
            f"Loaded map '{map_yaml}' -> {self.occ.width}x{self.occ.height}, "
            f"res={self.occ.metadata.resolution}, "
            f"origin=({self.occ.metadata.origin_x:.3f},{self.occ.metadata.origin_y:.3f})"
        )

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, scan_qos)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, odom_qos)

        self.latest_odom: Optional[Tuple[float, float, float, float]] = None  # x,y,yaw,stamp
        self.start_xy: Optional[Tuple[float, float]] = None
        self.last_sample_xy: Optional[Tuple[float, float]] = None
        self.last_sample_stamp: Optional[float] = None
        self.rows_written = 0
        self._open_csv()
        self.get_logger().info(
            f"Subscribed scan={self.scan_topic} odom={self.odom_topic}, "
            f"sample_step={self.sample_step_m}m min_dt={self.min_dt_sec}s, "
            f"output={self.output_csv}"
        )
        self.get_logger().info(
            "Teleop slowly along the route (odom-truth baseline). "
            "Stop the node with Ctrl-C to flush the CSV."
        )

    def _open_csv(self) -> None:
        write_header = not os.path.exists(self.output_csv) or os.path.getsize(self.output_csv) == 0
        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        self._csv_handle = open(self.output_csv, "a", encoding="utf-8")
        if write_header:
            self._csv_handle.write(",".join(CSV_HEADER) + "\n")
            self._csv_handle.flush()

    def _on_odom(self, msg: Odometry) -> None:
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        stamp = _stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec else _stamp_to_sec(self.get_clock().now().to_msg())
        self.latest_odom = (px, py, yaw, stamp)
        if self.start_xy is None:
            self.start_xy = (px, py)
            self.last_sample_xy = (px, py)
            self.last_sample_stamp = stamp

    def _on_scan(self, msg: LaserScan) -> None:
        if self.latest_odom is None:
            return
        ox, oy, oyaw, _ = self.latest_odom

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        finite = np.isfinite(ranges)
        beams_total = int(ranges.size)
        beams_finite = int(finite.sum())
        finite_ratio = (beams_finite / beams_total) if beams_total else 0.0

        residuals: List[float] = []
        measured: List[float] = []
        predicted: List[float] = []
        map_hits = 0
        offmap_or_unknown = 0
        rmax = float(msg.range_max)
        a0 = float(msg.angle_min)
        ai = float(msg.angle_increment)
        for idx in range(0, beams_total, self.beam_stride):
            if not finite[idx]:
                continue
            meas = float(ranges[idx])
            if not (float(msg.range_min) <= meas <= rmax):
                continue
            beam_local = a0 + idx * ai
            beam_world = oyaw + beam_local
            pred, end_state = _raycast_predicted_range(
                self.occ, ox, oy, oyaw, beam_world, rmax
            )
            measured.append(meas)
            predicted.append(pred)
            res = abs(meas - pred)
            if res > self.residual_clip_m:
                res = self.residual_clip_m
            residuals.append(res)
            if end_state == 2:
                map_hits += 1
            else:
                offmap_or_unknown += 1

        denom = max(1, len(residuals))
        res_mean = float(statistics.fmean(residuals)) if residuals else float("nan")
        res_median = float(statistics.median(residuals)) if residuals else float("nan")
        res_p90 = _percentile(residuals, 90.0)
        mean_meas = float(statistics.fmean(measured)) if measured else float("nan")
        mean_pred = float(statistics.fmean(predicted)) if predicted else float("nan")

        sx, sy = self.start_xy or (ox, oy)
        dist_from_start = math.hypot(ox - sx, oy - sy)

        # Throttle: travelled >= sample_step_m AND dt >= min_dt_sec.
        lx, ly = self.last_sample_xy or (ox, oy)
        moved = math.hypot(ox - lx, oy - ly)
        now_stamp = _stamp_to_sec(self.get_clock().now().to_msg())
        dt = (now_stamp - self.last_sample_stamp) if self.last_sample_stamp is not None else self.min_dt_sec
        if moved < self.sample_step_m and dt < self.min_dt_sec:
            return

        row = [
            f"{now_stamp:.3f}",
            f"{ox:.4f}", f"{oy:.4f}", f"{oyaw:.4f}",
            f"{dist_from_start:.3f}",
            str(beams_total), str(beams_finite), f"{finite_ratio:.4f}",
            f"{res_mean:.4f}", f"{res_median:.4f}", f"{res_p90:.4f}",
            f"{(map_hits / denom):.4f}", f"{(offmap_or_unknown / denom):.4f}",
            f"{mean_meas:.4f}", f"{mean_pred:.4f}",
        ]
        self._csv_handle.write(",".join(row) + "\n")
        self._csv_handle.flush()
        self.rows_written += 1
        self.last_sample_xy = (ox, oy)
        self.last_sample_stamp = now_stamp
        if self.rows_written % 10 == 0:
            self.get_logger().info(
                f"row {self.rows_written}: dist={dist_from_start:5.1f}m "
                f"finite={finite_ratio:4.2f} res_mean={res_mean:5.3f}m "
                f"res_p90={res_p90:5.3f}m map_hit={map_hits}/{denom}"
            )

    def destroy_node(self) -> bool:
        try:
            if self._csv_handle and not self._csv_handle.closed:
                self._csv_handle.flush()
                self._csv_handle.close()
        except Exception:
            pass
        self.get_logger().info(f"CSV flushed: {self.output_csv} ({self.rows_written} rows)")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ScanMapResidualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
