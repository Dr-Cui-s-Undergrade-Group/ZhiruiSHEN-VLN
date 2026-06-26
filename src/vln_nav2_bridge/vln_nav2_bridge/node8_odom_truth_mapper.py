#!/usr/bin/env python3
"""Odom-truth occupancy grid painter.

Builds a geometrically-exact occupancy map by accumulating /scan rays at the
TRUE robot pose (/chassis/odom), with NO SLAM localization. This sidesteps both
Phase 2 bottlenecks: (1) no AMCL drift, (2) each observed cell is marked
directly from its true ray geometry.

For every /scan at odom pose (x,y,yaw):
  - Bresenham raytrace each finite beam through the grid, marking cells FREE
  - mark the beam endpoint cell OCCUPIED
  - accumulate free/occupied hit counts per cell -> log-odds probability

On shutdown (Ctrl-C) saves PGM+YAML map. Resolution/origin match the original
warehouse_map so coordinates stay aligned with all target definitions.

Run under odom-truth baseline (no AMCL needed). Drive around to build coverage.
"""
from __future__ import annotations

import math
import os
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


# Grid params (aligned with original warehouse_map.yaml extents)
RESOLUTION = 0.05
ORIGIN_X = -10.11476
ORIGIN_Y = -19.8276
GRID_W = 480  # cells, matches original 480x777
GRID_H = 777
OUTPUT_PGM = ""
OUTPUT_YAML = ""
FREE_THRESHOLD = 0.196
OCC_THRESHOLD = 0.65


def _stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class OdomTruthMapPainter(Node):
    def __init__(self) -> None:
        super().__init__("node8_odom_truth_mapper")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("output_basename", "")
        self.declare_parameter("load_existing_map", True)
        self.declare_parameter("resolution", RESOLUTION)
        self.declare_parameter("origin_x", ORIGIN_X)
        self.declare_parameter("origin_y", ORIGIN_Y)
        self.declare_parameter("grid_width", GRID_W)
        self.declare_parameter("grid_height", GRID_H)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        base = str(self.get_parameter("output_basename").value)
        if not base:
            base = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..", "..", "..", "..", "data", "warehouse_map_truth",
            )
            base = os.path.normpath(base)
        self.out_pgm = base + ".pgm"
        self.out_yaml = base + ".yaml"
        self.res = float(self.get_parameter("resolution").value)
        self.ox = float(self.get_parameter("origin_x").value)
        self.oy = float(self.get_parameter("origin_y").value)
        self.gw = int(self.get_parameter("grid_width").value)
        self.gh = int(self.get_parameter("grid_height").value)

        # Accumulators: free_hits/occ_hits per cell
        self.free_hits = np.zeros((self.gh, self.gw), dtype=np.int32)
        self.occ_hits = np.zeros((self.gh, self.gw), dtype=np.int32)
        self.scans_processed = 0

        # Load existing map if available (incremental merge)
        self.load_existing = bool(self.get_parameter("load_existing_map").value)
        if self.load_existing and os.path.exists(self.out_pgm):
            self._load_existing_map()

        sq = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
        oq = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, sq)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, oq)

        self.latest_odom = None
        self.get_logger().info(
            f"Odom-truth mapper: scan={self.scan_topic} odom={self.odom_topic} "
            f"grid={self.gw}x{self.gh} res={self.res} "
            f"origin=({self.ox:.2f},{self.oy:.2f}) out={self.out_pgm}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.latest_odom = (p.x, p.y, math.atan2(siny, cosy))

    def _world_to_cell(self, wx: float, wy: float):
        col = int(round((wx - self.ox) / self.res))
        row = int(round((wy - self.oy) / self.res))
        return col, row

    def _on_scan(self, msg: LaserScan) -> None:
        if self.latest_odom is None:
            return
        ox, oy, oyaw = self.latest_odom
        ranges = msg.ranges
        a0 = float(msg.angle_min)
        ai = float(msg.angle_increment)
        rmin = float(msg.range_min)
        rmax = float(msg.range_max)
        gw, gh = self.gw, self.gh

        for i in range(len(ranges)):
            r = ranges[i]
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            angle = oyaw + a0 + i * ai
            ex = ox + r * math.cos(angle)
            ey = oy + r * math.sin(angle)
            self._trace_ray(ox, oy, ex, ey)

        self.scans_processed += 1
        if self.scans_processed % 50 == 0:
            occupied_cells = int((self.occ_hits > 0).sum())
            self.get_logger().info(
                f"scans={self.scans_processed} occupied_cells={occupied_cells} "
                f"odom=({ox:.1f},{oy:.1f})"
            )

    def _trace_ray(self, x0: float, y0: float, x1: float, y1: float) -> None:
        """Bresenham in grid coords: mark free along path, occupied at endpoint."""
        c0, r0 = self._world_to_cell(x0, y0)
        c1, r1 = self._world_to_cell(x1, y1)
        # Clip to grid bounds
        if not (0 <= c1 < self.gw and 0 <= r1 < self.gh):
            # endpoint off-grid: still trace the free portion within bounds
            c1 = max(0, min(self.gw - 1, c1))
            r1 = max(0, min(self.gh - 1, r1))
            endpoint_in = False
        else:
            endpoint_in = True

        dx = abs(c1 - c0)
        dy = abs(r1 - r0)
        sx = 1 if c0 < c1 else -1
        sy = 1 if r0 < r1 else -1
        err = dx - dy
        c, r = c0, r0
        while True:
            if c == c1 and r == r1:
                break
            if 0 <= c < self.gw and 0 <= r < self.gh:
                self.free_hits[r, c] += 1
            # stop tracing if we left the grid
            if not (0 <= c < self.gw and 0 <= r < self.gh):
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                c += sx
            if e2 < dx:
                err += dx
                r += sy
        if endpoint_in:
            self.occ_hits[r1, c1] += 1

    def _load_existing_map(self) -> None:
        """Load an existing PGM map and seed free_hits/occ_hits so new scans merge."""
        try:
            from PIL import Image as PILImage
            img = PILImage.open(self.out_pgm)
            if img.mode != "L":
                img = img.convert("L")
            w, h = img.size
            if w != self.gw or h != self.gh:
                self.get_logger().warn(
                    f"Existing map size {w}x{h} != grid {self.gw}x{self.gh}, skipping load"
                )
                return
            # PGM row 0 = top = max-y. Flip to our convention (row 0 = min-y).
            pixels = np.flipud(np.asarray(img, dtype=np.uint8))
            # 0=occupied, 254=free, 205=unknown (standard ROS)
            # Seed: occupied cells get occ_hits=5, free cells get free_hits=5
            # (weight 5 so they survive unless many new observations override)
            occ_mask = pixels < 50       # 0 = occupied
            free_mask = pixels > 220     # 254 = free (excludes 205 = unknown)
            self.occ_hits[occ_mask] = 5
            self.free_hits[free_mask] = 5
            n_occ = int(occ_mask.sum())
            n_free = int(free_mask.sum())
            self.get_logger().info(
                f"Loaded existing map: {self.out_pgm} "
                f"(seeded {n_occ} occupied + {n_free} free cells)"
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not load existing map: {exc}")

    def save_map(self) -> None:
        """Convert hit counts to occupancy probability and save PGM+YAML."""
        total = self.free_hits + self.occ_hits
        # P(map|obs) = occupied_hits / total_hits; unknown where total==0
        with np.errstate(divide="ignore", invalid="ignore"):
            prob = np.where(total > 0, self.occ_hits / total, -1.0)

        # PGM: 0=occupied, 254=free, 205=unknown (standard ROS occupancy)
        h, w = prob.shape
        pgm = np.full((h, w), 205, dtype=np.uint8)  # unknown default
        free_mask = (prob >= 0) & (prob < FREE_THRESHOLD)
        occ_mask = (prob >= 0) & (prob > OCC_THRESHOLD)
        pgm[free_mask] = 254
        pgm[occ_mask] = 0

        # PGM is stored with row 0 = TOP. Flip so row 0 = min-y (matches
        # warehouse_map convention where y increases downward in image).
        pgm_flipped = np.flipud(pgm)

        # Write PGM
        with open(self.out_pgm, "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode())
            f.write(pgm_flipped.tobytes())

        # Write YAML
        yaml_dir = os.path.dirname(os.path.abspath(self.out_yaml))
        pgm_name = os.path.basename(self.out_pgm)
        with open(self.out_yaml, "w") as f:
            f.write(f"image: {pgm_name}\n")
            f.write(f"resolution: {self.res}\n")
            f.write(f"origin: [{self.ox}, {self.oy}, 0.0]\n")
            f.write(f"negate: 0\n")
            f.write(f"occupied_thresh: {OCC_THRESHOLD}\n")
            f.write(f"free_thresh: {FREE_THRESHOLD}\n")

        occ_cells = int((prob > OCC_THRESHOLD).sum())
        free_cells = int((prob < FREE_THRESHOLD).sum())
        unk_cells = int((prob < 0).sum())
        self.get_logger().info(
            f"MAP SAVED: {self.out_pgm}  occupied={occ_cells} free={free_cells} "
            f"unknown={unk_cells} scans={self.scans_processed}"
        )


def main() -> None:
    rclpy.init()
    node = OdomTruthMapPainter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
