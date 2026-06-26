#!/usr/bin/env python3
"""Plot scan-vs-map residual diagnostic from node8_scan_map_residual output.

Produces two figures:
  1. node8_scan_map_residual_heatmap.png  - robot path overlaid on the static
     warehouse map, each sample colored by residual_mean_m (green->red).
  2. node8_scan_map_residual_curve.png    - residual_mean / res_p90 / finite_ratio
     vs distance-from-start, to separate plant (near) vs shelf (far) segments.

Usage:
  python3 src/analysis/plot_scan_map_residual.py
  python3 src/analysis/plot_scan_map_residual.py --csv data/X.csv --map data/warehouse_map.yaml
"""
import argparse
import csv
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "data"
ASSETS_DIR = ROOT / "assets"
DEFAULT_CSV = DATA_DIR / "node8_scan_map_residual_diagnostic.csv"
DEFAULT_MAP_YAML = DATA_DIR / "warehouse_map.yaml"

WIDTH = 1400
HEIGHT = 900
BG = (255, 255, 255)
TEXT = (31, 41, 55)
MUTED = (107, 114, 128)
GRID = (229, 231, 235)
AXIS = (156, 163, 175)
GOOD = (22, 163, 74)      # low residual (green)
WARN = (234, 88, 12)      # mid residual (orange)
BAD = (220, 38, 38)       # high residual (red)
PATH_INK = (37, 99, 235)  # blue


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


TITLE = font(34, bold=True)
SUBTITLE = font(20)
LABEL = font(18)
SMALL = font(15)
TINY = font(13)


def save(img: Image.Image, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)
    print(path.relative_to(ROOT))


def _load_map_yaml(map_yaml: Path) -> Tuple[Image.Image, float, float, float]:
    """Return (RGBA image, resolution, origin_x, origin_y)."""
    values = {}
    for raw in map_yaml.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        k, v = line.split(":", 1)
        values[k.strip()] = v.strip()
    image_path = values["image"]
    if not Path(image_path).is_absolute():
        image_path = str(map_yaml.parent / image_path)
    img = Image.open(image_path).convert("RGBA")
    res = float(values["resolution"])
    origin = [float(x) for x in values["origin"].strip().strip("[]").split(",")]
    return img, res, origin[0], origin[1]


def _world_to_pixel(x: float, y: float, res: float, ox: float, oy: float,
                    w: int, h: int) -> Tuple[int, int]:
    col = int(round((x - ox) / res))
    # YAML row 0 is top of image; flip so that increasing y -> up on map.
    row = int(round((h - 1) - ((y - oy) / res)))
    return col, row


def _lerp_color(t: float) -> Tuple[int, int, int]:
    """t in [0,1]: 0 green -> 0.5 orange -> 1 red."""
    t = max(0.0, min(1.0, t))
    if t <= 0.5:
        f = t / 0.5
        return (
            int(GOOD[0] + (WARN[0] - GOOD[0]) * f),
            int(GOOD[1] + (WARN[1] - GOOD[1]) * f),
            int(GOOD[2] + (WARN[2] - GOOD[2]) * f),
        )
    f = (t - 0.5) / 0.5
    return (
        int(WARN[0] + (BAD[0] - WARN[0]) * f),
        int(WARN[1] + (BAD[1] - WARN[1]) * f),
        int(WARN[2] + (BAD[2] - WARN[2]) * f),
    )


def read_csv(path: Path) -> List[dict]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _to_float(v: str) -> Optional[float]:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if math.isnan(f) or math.isinf(f):
        return None
    return f


def plot_heatmap(rows: List[dict], map_yaml: Path, csv_path: Path) -> None:
    img, res, ox, oy = _load_map_yaml(map_yaml)
    w, h = img.size

    # Drawn map canvas with a margin for legend/axes.
    margin = 60
    canvas = Image.new("RGB", (w + 2 * margin, h + 2 * margin), BG)
    canvas.paste(img, (margin, margin))

    draw = ImageDraw.Draw(canvas, "RGBA")

    # Determine residual scale from data (clamp to [0, vmax]).
    res_vals = [_to_float(r.get("residual_mean_m")) for r in rows]
    res_vals = [v for v in res_vals if v is not None]
    vmax = max(res_vals) if res_vals else 1.0
    vmax = max(0.3, min(vmax, 3.0))  # sensible bounds

    # Draw path: lines between consecutive samples, colored by residual at the segment start.
    prev_px: Optional[Tuple[int, int]] = None
    for r in rows:
        rx = _to_float(r.get("odom_x"))
        ry = _to_float(r.get("odom_y"))
        rv = _to_float(r.get("residual_mean_m"))
        if rx is None or ry is None:
            prev_px = None
            continue
        col, row = _world_to_pixel(rx, ry, res, ox, oy, w, h)
        px = (col + margin, row + margin)
        color = _lerp_color((rv / vmax) if (rv is not None and vmax > 0) else 0.0)
        if prev_px is not None:
            draw.line([prev_px, px], fill=color, width=5)
        r_radius = 6 if rv is not None else 4
        draw.ellipse(
            [px[0] - r_radius, px[1] - r_radius, px[0] + r_radius, px[1] + r_radius],
            fill=color, outline=(255, 255, 255),
        )
        prev_px = px

    # Mark origin and shelf target for reference.
    for name, (wx, wy) in [("origin", (0.0, 0.0)), ("purple boxes", (-6.3, 10.8))]:
        col, row = _world_to_pixel(wx, wy, res, ox, oy, w, h)
        px = (col + margin, row + margin)
        draw.ellipse([px[0] - 9, px[1] - 9, px[0] + 9, px[1] + 9],
                     outline=(255, 255, 255), width=2, fill=(37, 99, 235))
        draw.text((px[0] + 12, px[1] - 8), name, fill=TEXT, font=SMALL)

    # Legend strip (color bar) on the right.
    bar_x = w + margin + 10
    bar_y0 = margin
    bar_h = h
    bar_w = 22
    for i in range(bar_h):
        t = 1.0 - (i / max(1, bar_h - 1))
        draw.line([(bar_x, bar_y0 + i), (bar_x + bar_w, bar_y0 + i)],
                  fill=_lerp_color(t), width=1)
    draw.rectangle([bar_x, bar_y0, bar_x + bar_w, bar_y0 + bar_h],
                   outline=GRID, width=1)
    for frac, label in [(1.0, f"{vmax:.2f}m"), (0.5, f"{vmax/2:.2f}m"), (0.0, "0.00m")]:
        ly = int(bar_y0 + (1.0 - frac) * bar_h)
        draw.line([(bar_x + bar_w, ly), (bar_x + bar_w + 5, ly)], fill=AXIS, width=1)
        draw.text((bar_x + bar_w + 9, ly - 8), label, fill=MUTED, font=TINY)
    draw.text((bar_x - 4, bar_y0 - 24), "residual_mean_m", fill=TEXT, font=SMALL)

    # Title.
    draw.text((margin, 18), "Node 8 scan-vs-map residual (odom-truth path)",
              fill=TEXT, font=TITLE)
    n = len(rows)
    mean_overall = (_to_float(rows[0].get("residual_mean_m")) if rows else None)
    subtitle = (f"{csv_path.name}  |  {n} samples  |  color scale 0..{vmax:.2f}m"
                f"  |  green=scan matches map, red=scan sees geometry the map lacks")
    draw.text((margin, 52), subtitle, fill=MUTED, font=SUBTITLE)

    save(canvas, ASSETS_DIR / "node8_scan_map_residual_heatmap.png")


def plot_curve(rows: List[dict], csv_path: Path) -> None:
    img = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(img, "RGBA")

    plot_x0, plot_y0 = 90, 150
    plot_x1, plot_y1 = WIDTH - 60, HEIGHT - 110

    dists = [_to_float(r.get("distance_from_start_m")) for r in rows]
    means = [_to_float(r.get("residual_mean_m")) for r in rows]
    p90s = [_to_float(r.get("residual_p90_m")) for r in rows]
    fins = [_to_float(r.get("finite_ratio")) for r in rows]

    valid_d = [d for d in dists if d is not None]
    if not valid_d:
        draw.text((WIDTH // 2 - 200, HEIGHT // 2), "No valid distance samples in CSV",
                  fill=BAD, font=LABEL)
        save(img, ASSETS_DIR / "node8_scan_map_residual_curve.png")
        return
    dmax = max(0.1, max(valid_d))

    res_all = [v for v in (means + p90s) if v is not None]
    rmax = max(res_all) if res_all else 1.0
    rmax = max(0.3, min(rmax, 3.0))

    def x_of(d: float) -> float:
        return plot_x0 + (d / dmax) * (plot_x1 - plot_x0)

    def y_res(v: float) -> float:
        return plot_y1 - (min(v, rmax) / rmax) * (plot_y1 - plot_y0)

    def y_fin(v: float) -> float:
        return plot_y1 - (max(0.0, min(v, 1.0))) * (plot_y1 - plot_y0)

    # Grid + axes.
    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        gx = int(plot_x0 + frac * (plot_x1 - plot_x0))
        draw.line([(gx, plot_y0), (gx, plot_y1)], fill=GRID, width=1)
        draw.text((gx - 14, plot_y1 + 8), f"{frac*dmax:.1f}m", fill=MUTED, font=TINY)
        gy = int(plot_y0 + frac * (plot_y1 - plot_y0))
        draw.line([(plot_x0, gy), (plot_x1, gy)], fill=GRID, width=1)
        draw.text((plot_x0 - 52, gy - 8), f"{(1-frac)*rmax:.2f}m", fill=MUTED, font=TINY)
    draw.rectangle([plot_x0, plot_y0, plot_x1, plot_y1], outline=AXIS, width=2)
    draw.text((plot_x0 - 50, plot_y0 - 28), "residual (m)", fill=TEXT, font=SMALL)
    draw.text(((plot_x0 + plot_x1) // 2 - 50, plot_y1 + 34),
              "distance from start (m)", fill=TEXT, font=SMALL)

    # finite_ratio as faint background band (right axis 0..1).
    fin_pts = [(x_of(d), y_fin(f)) for d, f in zip(dists, fins)
               if d is not None and f is not None]
    if len(fin_pts) >= 2:
        draw.line(fin_pts, fill=(37, 99, 235), width=2)

    # p90 (orange), mean (red).
    p90_pts = [(x_of(d), y_res(v)) for d, v in zip(dists, p90s)
               if d is not None and v is not None]
    mean_pts = [(x_of(d), y_res(v)) for d, v in zip(dists, means)
                if d is not None and v is not None]
    if len(p90_pts) >= 2:
        draw.line(p90_pts, fill=WARN, width=2)
    if len(mean_pts) >= 2:
        draw.line(mean_pts, fill=BAD, width=3)
    for px, py in mean_pts:
        draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill=BAD)

    # Legend.
    lx = plot_x1 - 250
    ly = plot_y0 + 10
    draw.line([(lx, ly + 8), (lx + 30, ly + 8)], fill=BAD, width=3)
    draw.text((lx + 38, ly), "residual_mean_m", fill=TEXT, font=TINY)
    draw.line([(lx, ly + 32), (lx + 30, ly + 32)], fill=WARN, width=2)
    draw.text((lx + 38, ly + 24), "residual_p90_m", fill=TEXT, font=TINY)
    draw.line([(lx, ly + 56), (lx + 30, ly + 56)], fill=(37, 99, 235), width=2)
    draw.text((lx + 38, ly + 48), "finite_ratio (0..1, right axis)", fill=TEXT, font=TINY)

    # Title + verdict hint.
    draw.text((48, 34), "Node 8 scan-vs-map residual along route",
              fill=TEXT, font=TITLE)
    verdict = ("shelf/corridor segment (far) high vs plant (near) low  ->  static map is culprit"
               if _segment_split(dists, means) else "")
    draw.text((50, 78),
              f"{csv_path.name}  |  rising residual toward the shelf implies the static map is too coarse there",
              fill=MUTED, font=SUBTITLE)
    if verdict:
        draw.text((50, 108), verdict, fill=GOOD, font=SMALL)

    save(img, ASSETS_DIR / "node8_scan_map_residual_curve.png")


def _segment_split(dists: List[Optional[float]],
                   means: List[Optional[float]]) -> bool:
    """Heuristic: is mean residual in the far half of the route clearly higher
    than the near half? Used only to hint a verdict in the plot subtitle."""
    near, far = [], []
    valid = [(d, v) for d, v in zip(dists, means) if d is not None and v is not None]
    if len(valid) < 6:
        return False
    dmax = max(d for d, _ in valid)
    for d, v in valid:
        (near if d < dmax * 0.5 else far).append(v)
    if not near or not far:
        return False
    return (sum(far) / len(far)) > 1.5 * (sum(near) / len(near))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", default=str(DEFAULT_CSV))
    parser.add_argument("--map", default=str(DEFAULT_MAP_YAML))
    args = parser.parse_args()

    csv_path = Path(args.csv)
    map_yaml = Path(args.map)
    if not csv_path.exists():
        sys.exit(f"CSV not found: {csv_path}. Run node8_scan_map_residual first.")
    if not map_yaml.exists():
        sys.exit(f"Map yaml not found: {map_yaml}")

    rows = read_csv(csv_path)
    if not rows:
        sys.exit(f"CSV has no data rows: {csv_path}")

    plot_heatmap(rows, map_yaml, csv_path)
    plot_curve(rows, csv_path)


if __name__ == "__main__":
    main()
