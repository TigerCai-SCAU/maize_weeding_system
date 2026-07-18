#!/usr/bin/env python3
"""Create YOLO detection labels for green fake seedlings on the gray bench belt."""

import argparse
import csv
from pathlib import Path
import shutil

import cv2
import numpy as np


def belt_roi(height: int, width: int) -> np.ndarray:
    """Return the interior of the conveyor belt, excluding rails and workshop clutter."""
    polygon = np.array(
        [
            [int(0.315 * width), 0],
            [int(0.685 * width), 0],
            [int(0.785 * width), height - 1],
            [int(0.215 * width), height - 1],
        ],
        dtype=np.int32,
    )
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, polygon, 255)

    # Fixed bench hardware that contains green/yellow pixels.  Keep these
    # exclusions deliberately tight so seedlings in the two crop rows remain
    # inside the usable belt area.
    mask[
        int(0.82 * height) : height,
        int(0.75 * width) : int(0.82 * width),
    ] = 0
    return mask


def find_base_candidates(
    image: np.ndarray,
    green: np.ndarray,
) -> list[tuple[int, int, float]]:
    """Locate compact black bases before foliage instances are separated."""
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    dark_value_max = 78
    dark = cv2.inRange(value, 0, dark_value_max)
    dark = cv2.bitwise_and(dark, belt_roi(height, width))
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)),
    )
    distance_to_green = cv2.distanceTransform(
        cv2.bitwise_not(green), cv2.DIST_L2, 3
    )

    contours, _ = cv2.findContours(
        dark,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    candidates: list[tuple[int, int, float]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 45:
            continue
        cx, cy, cw, ch = cv2.boundingRect(contour)
        if cw < 7 or ch < 7:
            continue
        expected_diameter = 16.0 + 34.0 * (cy + ch / 2.0) / height
        expected_area = np.pi * (expected_diameter / 2.0) ** 2
        if area > 4.5 * expected_area:
            continue
        aspect = cw / ch
        fill = area / (cw * ch)
        if not 0.42 <= aspect <= 2.4 or fill < 0.28:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        point_x = round(moments["m10"] / moments["m00"])
        point_y = round(moments["m01"] / moments["m00"])
        green_distance = float(distance_to_green[point_y, point_x])
        if green_distance > 1.5 * expected_diameter:
            continue

        perimeter = cv2.arcLength(contour, True)
        circularity = (
            4.0 * np.pi * area / (perimeter * perimeter)
            if perimeter > 0
            else 0.0
        )
        contour_mask = np.zeros((ch, cw), np.uint8)
        shifted_contour = contour - np.array([[[cx, cy]]], dtype=contour.dtype)
        cv2.drawContours(contour_mask, [shifted_contour], -1, 255, cv2.FILLED)
        mean_value = cv2.mean(
            value[cy : cy + ch, cx : cx + cw],
            mask=contour_mask,
        )[0]
        area_penalty = abs(np.log(max(area, 1.0) / expected_area))
        score = (
            3.0 * circularity
            + 1.5 * fill
            + (dark_value_max - mean_value) / 40.0
            - area_penalty
            - green_distance / expected_diameter
        )
        candidates.append((point_x, point_y, score))

    candidates.sort(key=lambda candidate: candidate[2], reverse=True)
    kept: list[tuple[int, int, float]] = []
    for point_x, point_y, score in candidates:
        expected_diameter = 16.0 + 34.0 * point_y / height
        if any(
            (point_x - old_x) ** 2 + (point_y - old_y) ** 2
            < (0.55 * expected_diameter) ** 2
            for old_x, old_y, _ in kept
        ):
            continue
        kept.append((point_x, point_y, score))
    return kept


def recover_base_centers(
    image: np.ndarray,
    green: np.ndarray,
    boxes: list[tuple[int, int, int, int]],
    points: list[tuple[int, int] | None],
) -> list[tuple[int, int] | None]:
    """Recover bases whose dark region only becomes compact inside an instance crop."""
    height, width = image.shape[:2]
    value = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 2]
    dark_value_max = 78
    dark = cv2.inRange(value, 0, dark_value_max)
    dark = cv2.bitwise_and(dark, belt_roi(height, width))
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)),
    )
    distance_to_green = cv2.distanceTransform(
        cv2.bitwise_not(green), cv2.DIST_L2, 3
    )

    recovered = list(points)
    claimed = [point for point in recovered if point is not None]
    for index, ((x1, y1, x2, y2), point) in enumerate(zip(boxes, recovered)):
        if point is not None:
            continue
        pad = 20
        crop_x1 = max(0, x1 - pad)
        crop_y1 = max(0, y1 - pad)
        crop_x2 = min(width, x2 + pad)
        crop_y2 = min(height, y2 + pad)
        contours, _ = cv2.findContours(
            dark[crop_y1:crop_y2, crop_x1:crop_x2],
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        expected_diameter = 16.0 + 34.0 * ((y1 + y2) / 2.0) / height
        expected_area = np.pi * (expected_diameter / 2.0) ** 2
        best_score = float("-inf")
        best_point: tuple[int, int] | None = None
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 45 or area > 4.5 * expected_area:
                continue
            cx, cy, cw, ch = cv2.boundingRect(contour)
            if cw < 7 or ch < 7:
                continue
            aspect = cw / ch
            fill = area / (cw * ch)
            if not 0.42 <= aspect <= 2.4 or fill < 0.28:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            point_x = crop_x1 + round(moments["m10"] / moments["m00"])
            point_y = crop_y1 + round(moments["m01"] / moments["m00"])
            if any(
                (point_x - old_x) ** 2 + (point_y - old_y) ** 2
                < (0.55 * expected_diameter) ** 2
                for old_x, old_y in claimed
            ):
                continue
            green_distance = float(distance_to_green[point_y, point_x])
            if green_distance > 1.5 * expected_diameter:
                continue
            perimeter = cv2.arcLength(contour, True)
            circularity = (
                4.0 * np.pi * area / (perimeter * perimeter)
                if perimeter > 0
                else 0.0
            )
            contour_mask = np.zeros(
                (crop_y2 - crop_y1, crop_x2 - crop_x1),
                np.uint8,
            )
            cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)
            mean_value = cv2.mean(
                value[crop_y1:crop_y2, crop_x1:crop_x2],
                mask=contour_mask,
            )[0]
            score = (
                3.0 * circularity
                + 1.5 * fill
                + (dark_value_max - mean_value) / 40.0
                - abs(np.log(max(area, 1.0) / expected_area))
                - green_distance / expected_diameter
            )
            if score > best_score:
                best_score = score
                best_point = (point_x, point_y)
        if best_point is not None:
            recovered[index] = best_point
            claimed.append(best_point)
    return recovered


def padded_instance_box(
    xs: np.ndarray,
    ys: np.ndarray,
    point: tuple[int, int] | None,
    width: int,
    height: int,
) -> tuple[int, int, int, int] | None:
    """Build one YOLO box around foliage and its black base."""
    if xs.size < 100:
        return None
    x1 = int(xs.min())
    y1 = int(ys.min())
    x2 = int(xs.max()) + 1
    y2 = int(ys.max()) + 1

    if point is not None:
        point_x, point_y = point
        base_radius = round(8.0 + 17.0 * point_y / height)
        x1 = min(x1, point_x - base_radius)
        y1 = min(y1, point_y - base_radius)
        x2 = max(x2, point_x + base_radius + 1)
        y2 = max(y2, point_y + base_radius + 1)

    box_width = x2 - x1
    box_height = y2 - y1
    if box_width < 18 or box_height < 18:
        return None
    if box_width > int(0.16 * width) or box_height > int(0.24 * height):
        return None

    pad_x = max(12, int(0.10 * box_width))
    pad_top = max(8, int(0.06 * box_height))
    pad_bottom = max(24, int(0.18 * box_height))
    return (
        max(0, x1 - pad_x),
        max(0, y1 - pad_top),
        min(width, x2 + pad_x),
        min(height, y2 + pad_bottom),
    )


def detect_seedlings(
    image: np.ndarray,
) -> tuple[
    list[tuple[int, int, int, int]],
    list[tuple[int, int] | None],
    np.ndarray,
]:
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Green foliage. The belt and shadows have low saturation and are excluded.
    green = cv2.inRange(
        hsv,
        np.array([22, 48, 45], dtype=np.uint8),
        np.array([92, 255, 255], dtype=np.uint8),
    )
    green = cv2.bitwise_and(green, belt_roi(height, width))
    green = cv2.morphologyEx(
        green,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    green = cv2.morphologyEx(
        green,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)),
    )

    # Join the separated leaves belonging to one seedling.
    joined = cv2.dilate(
        green,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31)),
        iterations=1,
    )
    component_count, component_map, stats, _ = cv2.connectedComponentsWithStats(
        joined, connectivity=8
    )

    components: list[tuple[np.ndarray, np.ndarray]] = []
    for component_id in range(1, component_count):
        sx, sy, sw, sh, _ = stats[component_id]
        original_component = np.logical_and(
            component_map[sy : sy + sh, sx : sx + sw] == component_id,
            green[sy : sy + sh, sx : sx + sw] > 0,
        )
        yy, xx = np.nonzero(original_component)
        green_area = int(xx.size)
        if green_area < 260:
            continue
        components.append((sx + xx, sy + yy))

    base_candidates = find_base_candidates(image, green)
    candidates_by_component: list[list[tuple[int, int, float]]] = [
        [] for _ in components
    ]
    for candidate in base_candidates:
        if candidate[2] < 1.6:
            continue
        point_x, point_y, _ = candidate
        expected_diameter = 16.0 + 34.0 * point_y / height
        best_component = None
        best_distance = float("inf")
        for component_index, (xs, ys) in enumerate(components):
            distance = float(
                np.min((xs - point_x) ** 2 + (ys - point_y) ** 2)
            )
            if distance < best_distance:
                best_distance = distance
                best_component = component_index
        if (
            best_component is not None
            and best_distance <= (1.5 * expected_diameter) ** 2
        ):
            candidates_by_component[best_component].append(candidate)

    instances: list[
        tuple[tuple[int, int, int, int], tuple[int, int] | None]
    ] = []
    for (xs, ys), candidates in zip(components, candidates_by_component):
        if not candidates:
            box = padded_instance_box(xs, ys, None, width, height)
            if box is not None:
                instances.append((box, None))
            continue

        centers = np.array(
            [(point_x, point_y) for point_x, point_y, _ in candidates],
            dtype=np.float32,
        )
        pixels = np.column_stack((xs, ys)).astype(np.float32)
        ownership = np.argmin(
            np.sum(
                (pixels[:, np.newaxis, :] - centers[np.newaxis, :, :]) ** 2,
                axis=2,
            ),
            axis=1,
        )
        for candidate_index, (point_x, point_y, _) in enumerate(candidates):
            owned = ownership == candidate_index
            box = padded_instance_box(
                xs[owned],
                ys[owned],
                (point_x, point_y),
                width,
                height,
            )
            if box is not None:
                instances.append((box, (point_x, point_y)))

    instances.sort(key=lambda instance: (instance[0][1], instance[0][0]))
    boxes = [instance[0] for instance in instances]
    base_centers = [instance[1] for instance in instances]
    base_centers = recover_base_centers(
        image,
        green,
        boxes,
        base_centers,
    )
    return boxes, base_centers, green


def yolo_line(box: tuple[int, int, int, int], width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    center_x = (x1 + x2) / (2.0 * width)
    center_y = (y1 + y2) / (2.0 * height)
    box_width = (x2 - x1) / width
    box_height = (y2 - y1) / height
    return (
        f"0 {center_x:.6f} {center_y:.6f} "
        f"{box_width:.6f} {box_height:.6f}"
    )


def yolo_pose_line(
    box: tuple[int, int, int, int],
    point: tuple[int, int] | None,
    width: int,
    height: int,
) -> str:
    detection = yolo_line(box, width, height)
    if point is None:
        return f"{detection} 0.000000 0.000000 0"
    return (
        f"{detection} "
        f"{point[0] / width:.6f} {point[1] / height:.6f} 2"
    )


def draw_preview(
    image: np.ndarray,
    boxes: list[tuple[int, int, int, int]],
    points: list[tuple[int, int] | None],
    image_name: str,
) -> np.ndarray:
    preview = image.copy()
    for index, ((x1, y1, x2, y2), point) in enumerate(zip(boxes, points)):
        cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 0, 255), 5)
        if point is not None:
            cv2.drawMarker(
                preview,
                point,
                (255, 255, 0),
                cv2.MARKER_CROSS,
                28,
                5,
                cv2.LINE_AA,
            )
            cv2.circle(preview, point, 10, (255, 255, 0), 3, cv2.LINE_AA)
        else:
            cv2.putText(
                preview,
                "NO-POINT",
                (x1, min(preview.shape[0] - 10, y2 + 35)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 255),
                3,
                cv2.LINE_AA,
            )
        cv2.putText(
            preview,
            str(index + 1),
            (x1, max(35, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            3,
            cv2.LINE_AA,
        )
    cv2.putText(
        preview,
        f"{image_name}: {len(boxes)} maize",
        (35, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.5,
        (255, 0, 255),
        4,
        cv2.LINE_AA,
    )
    preview_width = 1440
    if preview.shape[1] > preview_width:
        scale = preview_width / preview.shape[1]
        preview = cv2.resize(
            preview,
            (preview_width, round(preview.shape[0] * scale)),
            interpolation=cv2.INTER_AREA,
        )
    return preview


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--pose-output-dir",
        help="Separate YOLO pose dataset root; defaults to OUTPUT_DIR + '_pose'",
    )
    parser.add_argument("--preview-stride", type=int, default=10)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    pose_output_dir = (
        Path(args.pose_output_dir).resolve()
        if args.pose_output_dir
        else Path(f"{output_dir}_pose")
    )
    images = sorted(input_dir.glob("*.jpg"))
    if not images:
        raise FileNotFoundError(f"No JPG images found in {input_dir}")

    split_at = max(1, min(len(images) - 1, round(len(images) * (1 - args.val_ratio))))
    for split in ("train", "val"):
        (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
        (pose_output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (pose_output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
    (output_dir / "previews").mkdir(parents=True, exist_ok=True)
    (output_dir / "flagged").mkdir(parents=True, exist_ok=True)

    rows = []
    for index, image_path in enumerate(images):
        split = "train" if index < split_at else "val"
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            rows.append((image_path.name, split, -1, -1, "read_error"))
            continue

        height, width = image.shape[:2]
        boxes, points, _ = detect_seedlings(image)
        label_path = output_dir / "labels" / split / f"{image_path.stem}.txt"
        label_path.write_text(
            "\n".join(yolo_line(box, width, height) for box in boxes),
            encoding="utf-8",
        )
        pose_label_path = (
            pose_output_dir / "labels" / split / f"{image_path.stem}.txt"
        )
        pose_label_path.write_text(
            "\n".join(
                yolo_pose_line(box, point, width, height)
                for box, point in zip(boxes, points)
            ),
            encoding="utf-8",
        )

        image_link = output_dir / "images" / split / image_path.name
        if not image_link.exists():
            image_link.symlink_to(image_path)
        pose_image_link = pose_output_dir / "images" / split / image_path.name
        if not pose_image_link.exists():
            pose_image_link.symlink_to(image_path)

        point_count = sum(point is not None for point in points)
        status = (
            "ok"
            if 7 <= len(boxes) <= 18 and point_count == len(boxes)
            else "review"
        )
        rows.append((image_path.name, split, len(boxes), point_count, status))
        if status == "review":
            shutil.copy2(image_path, output_dir / "flagged" / image_path.name)

        if index % args.preview_stride == 0 or status == "review":
            preview = draw_preview(image, boxes, points, image_path.name)
            cv2.imwrite(
                str(output_dir / "previews" / image_path.name),
                preview,
                [cv2.IMWRITE_JPEG_QUALITY, 88],
            )

    with (output_dir / "annotation_report.csv").open(
        "w", newline="", encoding="utf-8"
    ) as report_file:
        writer = csv.writer(report_file)
        writer.writerow(["image", "split", "box_count", "point_count", "status"])
        writer.writerows(rows)

    (output_dir / "dataset.yaml").write_text(
        f"path: {output_dir}\n"
        "train: images/train\n"
        "val: images/val\n\n"
        "names:\n"
        "  0: maize\n",
        encoding="utf-8",
    )
    (pose_output_dir / "dataset.yaml").write_text(
        f"path: {pose_output_dir}\n"
        "train: images/train\n"
        "val: images/val\n\n"
        "kpt_shape: [1, 3]\n"
        "flip_idx: [0]\n\n"
        "names:\n"
        "  0: maize\n",
        encoding="utf-8",
    )

    counts = [row[2] for row in rows if row[2] >= 0]
    point_counts = [row[3] for row in rows if row[3] >= 0]
    review_count = sum(row[4] != "ok" for row in rows)
    print(
        f"images={len(images)} train={split_at} val={len(images) - split_at} "
        f"boxes={sum(counts)} min={min(counts)} max={max(counts)} "
        f"mean={sum(counts) / len(counts):.2f} "
        f"points={sum(point_counts)} missing={sum(counts) - sum(point_counts)} "
        f"review={review_count}"
    )


if __name__ == "__main__":
    main()
