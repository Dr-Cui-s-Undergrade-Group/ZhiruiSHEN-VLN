#!/usr/bin/env python3
import csv
import math
from collections import Counter
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "data"
ASSETS_DIR = ROOT / "assets"
NODE6_CSV = DATA_DIR / "node6_auto_trials_2026-06-13_final.csv"
NODE7_CSV = DATA_DIR / "node7_ablation_2026-06-13.csv"


WIDTH = 1400
HEIGHT = 900
BG = (255, 255, 255)
TEXT = (31, 41, 55)
MUTED = (107, 114, 128)
GRID = (229, 231, 235)
SUCCESS = (22, 163, 74)
ARRIVED_ONLY = (234, 88, 12)
FAILED = (220, 38, 38)
BLUE = (37, 99, 235)
PURPLE = (124, 58, 237)


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


def read_csv(path: Path):
    with path.open(newline="", encoding="utf-8") as file_obj:
        return list(csv.DictReader(file_obj))


def save(img: Image.Image, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)
    print(path.relative_to(ROOT))


def draw_title(draw: ImageDraw.ImageDraw, title: str, subtitle: str) -> None:
    draw.text((48, 34), title, fill=TEXT, font=TITLE)
    draw.text((50, 80), subtitle, fill=MUTED, font=SUBTITLE)


def bool_value(value: str) -> bool:
    return str(value).strip().lower() in ("true", "1", "yes")


def plot_target_vs_final(rows) -> None:
    img = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(img)
    draw_title(
        draw,
        "Node 6 Target vs Final Robot Pose",
        "Green: bridge success; orange: physically arrived but strict confirmation failed; red: not arrived.",
    )

    left, top, right, bottom = 120, 150, 980, 790
    xs, ys = [], []
    points = []
    for row in rows:
        try:
            tx, ty = float(row["target_x"]), float(row["target_y"])
            fx, fy = float(row["final_x"]), float(row["final_y"])
        except (KeyError, TypeError, ValueError):
            continue
        xs.extend([tx, fx])
        ys.extend([ty, fy])
        points.append((row, tx, ty, fx, fy))

    min_x, max_x = math.floor(min(xs) - 1), math.ceil(max(xs) + 1)
    min_y, max_y = math.floor(min(ys) - 1), math.ceil(max(ys) + 1)

    def sx(x):
        return left + (x - min_x) / (max_x - min_x) * (right - left)

    def sy(y):
        return bottom - (y - min_y) / (max_y - min_y) * (bottom - top)

    draw.rectangle((left, top, right, bottom), outline=(156, 163, 175), width=2)
    for i in range(min_x, max_x + 1, 2):
        x = sx(i)
        draw.line((x, top, x, bottom), fill=GRID, width=1)
        draw.text((x - 12, bottom + 10), str(i), fill=MUTED, font=TINY)
    for i in range(min_y, max_y + 1, 2):
        y = sy(i)
        draw.line((left, y, right, y), fill=GRID, width=1)
        draw.text((left - 44, y - 8), str(i), fill=MUTED, font=TINY)

    draw.text(((left + right) // 2 - 45, bottom + 45), "map x (m)", fill=MUTED, font=SMALL)
    draw.text((32, (top + bottom) // 2), "map y (m)", fill=MUTED, font=SMALL)

    for row, tx, ty, fx, fy in points:
        trial = row["trial_id"]
        arrived = bool_value(row["within_success_radius"])
        bridge_success = row["nav_result"] == "success"
        color = SUCCESS if bridge_success else ARRIVED_ONLY if arrived else FAILED
        draw.line((sx(tx), sy(ty), sx(fx), sy(fy)), fill=color, width=2)
        draw.ellipse((sx(tx) - 7, sy(ty) - 7, sx(tx) + 7, sy(ty) + 7), outline=BLUE, width=3)
        draw.rectangle((sx(fx) - 6, sy(fy) - 6, sx(fx) + 6, sy(fy) + 6), fill=color)
        draw.text((sx(fx) + 8, sy(fy) - 10), trial, fill=TEXT, font=TINY)

    legend_x, legend_y = 1040, 180
    legend_items = [
        ("Target coordinate", BLUE, "circle"),
        ("Bridge success", SUCCESS, "square"),
        ("Arrived, strict fail", ARRIVED_ONLY, "square"),
        ("Not arrived", FAILED, "square"),
    ]
    draw.text((legend_x, legend_y - 42), "Legend", fill=TEXT, font=LABEL)
    for idx, (name, color, shape) in enumerate(legend_items):
        y = legend_y + idx * 42
        if shape == "circle":
            draw.ellipse((legend_x, y, legend_x + 18, y + 18), outline=color, width=3)
        else:
            draw.rectangle((legend_x, y, legend_x + 18, y + 18), fill=color)
        draw.text((legend_x + 32, y - 2), name, fill=TEXT, font=SMALL)

    summary = [
        "Key readout",
        "Bridge success: 7/15",
        "Physical arrival: 11/15",
        "Trials 8-11 reached the",
        "shelf/package target region",
        "but failed strict visual",
        "confirmation.",
    ]
    y = 420
    for idx, text in enumerate(summary):
        draw.text((1040, y), text, fill=TEXT if idx == 0 else MUTED, font=LABEL if idx == 0 else SMALL)
        y += 30

    save(img, ASSETS_DIR / "node6_target_vs_final_pose.png")


def plot_failure_taxonomy(rows) -> None:
    img = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(img)
    draw_title(
        draw,
        "Node 6 Failure Taxonomy",
        "Failures are counted from the strict Node 6 bridge result.",
    )

    failed_rows = [row for row in rows if row["nav_result"] != "success"]
    categories = Counter()
    for row in failed_rows:
        method = row["parse_method"]
        if method == "semantic_explore_visual_scan_failed":
            categories["Strict shelf/package confirmation"] += 1
        elif method == "visual_map_failed":
            categories["Semantic alias / map mismatch"] += 1
        elif method == "visual_scan_failed":
            categories["Visual scan unavailable"] += 1
        else:
            categories[method] += 1

    items = [
        ("Strict shelf/package confirmation", categories["Strict shelf/package confirmation"], ARRIVED_ONLY),
        ("Semantic alias / map mismatch", categories["Semantic alias / map mismatch"], PURPLE),
        ("Visual scan unavailable", categories["Visual scan unavailable"], FAILED),
    ]

    left, top = 150, 180
    bar_max_w = 900
    bar_h = 70
    max_count = max(count for _, count, _ in items)
    for idx, (name, count, color) in enumerate(items):
        y = top + idx * 145
        draw.text((left, y - 32), name, fill=TEXT, font=LABEL)
        draw.rectangle((left, y, left + bar_max_w, y + bar_h), outline=GRID, width=2)
        w = int(bar_max_w * count / max_count)
        draw.rectangle((left, y, left + w, y + bar_h), fill=color)
        draw.text((left + w + 18, y + 19), f"{count} trials", fill=TEXT, font=LABEL)

    notes = [
        "Interpretation",
        "The largest strict-failure group is not a planning failure.",
        "Trials 8-11 physically arrived, then failed because the",
        "confirmation logic rejected equivalent shelf/package evidence.",
        "Node 7 directly targets this category.",
    ]
    y = 650
    for idx, text in enumerate(notes):
        draw.text((150, y), text, fill=TEXT if idx == 0 else MUTED, font=LABEL if idx == 0 else SMALL)
        y += 32

    save(img, ASSETS_DIR / "node6_failure_taxonomy.png")


def plot_ablation(rows) -> None:
    img = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(img)
    draw_title(
        draw,
        "Node 7 Ablation Comparison",
        "Offline metric and confirmation-logic ablation over the fixed 15-trial Node 6 final CSV.",
    )

    variants = [row["variant"] for row in rows]
    labels = ["Bridge success", "Navigation arrived", "Visual confirmed", "Task success"]
    fields = ["bridge_success", "navigation_arrived", "visual_confirmed", "task_success"]
    colors = [BLUE, SUCCESS, PURPLE, ARRIVED_ONLY]

    chart_left, chart_top, chart_right, chart_bottom = 120, 180, 1200, 700
    max_value = 15
    for tick in range(0, max_value + 1, 3):
        y = chart_bottom - tick / max_value * (chart_bottom - chart_top)
        draw.line((chart_left, y, chart_right, y), fill=GRID, width=1)
        draw.text((chart_left - 36, y - 10), str(tick), fill=MUTED, font=TINY)
    draw.text((50, 425), "trials", fill=MUTED, font=SMALL)

    group_w = (chart_right - chart_left) / len(rows)
    bar_w = 44
    gap = 8
    for group_idx, row in enumerate(rows):
        base_x = chart_left + group_idx * group_w + 72
        for idx, field in enumerate(fields):
            value = int(row[field])
            x0 = base_x + idx * (bar_w + gap)
            y0 = chart_bottom - value / max_value * (chart_bottom - chart_top)
            draw.rectangle((x0, y0, x0 + bar_w, chart_bottom), fill=colors[idx])
            draw.text((x0 + 9, y0 - 24), str(value), fill=TEXT, font=TINY)
        name = variants[group_idx].replace("_", " ")
        wrapped = name.replace("metric split plus", "metric split +")
        draw.text((base_x - 20, chart_bottom + 24), wrapped, fill=TEXT, font=TINY)

    legend_x, legend_y = 120, 760
    for idx, label in enumerate(labels):
        x = legend_x + idx * 280
        draw.rectangle((x, legend_y, x + 22, legend_y + 22), fill=colors[idx])
        draw.text((x + 32, legend_y - 2), label, fill=TEXT, font=SMALL)

    save(img, ASSETS_DIR / "node7_ablation_comparison.png")


def main() -> None:
    node6_rows = read_csv(NODE6_CSV)
    node7_rows = read_csv(NODE7_CSV)
    plot_target_vs_final(node6_rows)
    plot_failure_taxonomy(node6_rows)
    plot_ablation(node7_rows)


if __name__ == "__main__":
    main()
