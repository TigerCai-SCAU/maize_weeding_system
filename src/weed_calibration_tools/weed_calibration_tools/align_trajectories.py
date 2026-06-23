import argparse
import math
from pathlib import Path

import numpy as np


def load_tum(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals = [float(v) for v in line.split()]
            if len(vals) != 8:
                raise RuntimeError(f"Bad TUM line in {path}: {line}")
            rows.append(vals)
    if len(rows) < 3:
        raise RuntimeError(f"Need at least 3 poses in {path}")
    return np.asarray(rows, dtype=np.float64)


def associate(source, target, max_dt):
    pairs = []
    j = 0
    for i in range(source.shape[0]):
        t = source[i, 0]
        while j + 1 < target.shape[0] and target[j + 1, 0] < t:
            j += 1
        candidates = [j]
        if j + 1 < target.shape[0]:
            candidates.append(j + 1)
        best = min(candidates, key=lambda k: abs(target[k, 0] - t))
        dt = abs(target[best, 0] - t)
        if dt <= max_dt:
            pairs.append((i, best, dt))
    if len(pairs) < 3:
        raise RuntimeError(f"Only matched {len(pairs)} pose pairs; increase --max-dt or check timestamps")
    return pairs


def timestamp_range(traj):
    return float(traj[0, 0]), float(traj[-1, 0])


def apply_time_offset(traj, offset):
    shifted = traj.copy()
    shifted[:, 0] += offset
    return shifted


def count_pairs(source, target, max_dt):
    j = 0
    count = 0
    total_dt = 0.0
    max_seen_dt = 0.0
    for i in range(source.shape[0]):
        t = source[i, 0]
        while j + 1 < target.shape[0] and target[j + 1, 0] < t:
            j += 1
        candidates = [j]
        if j + 1 < target.shape[0]:
            candidates.append(j + 1)
        best_dt = min(abs(target[k, 0] - t) for k in candidates)
        if best_dt <= max_dt:
            count += 1
            total_dt += best_dt
            max_seen_dt = max(max_seen_dt, best_dt)
    mean_dt = total_dt / count if count else float("inf")
    return count, mean_dt, max_seen_dt


def search_time_offset(source, target, max_dt, search_range, search_step):
    if search_range <= 0.0 or search_step <= 0.0:
        return 0.0, count_pairs(source, target, max_dt)
    best = None
    steps = int(round((2.0 * search_range) / search_step))
    for n in range(steps + 1):
        offset = -search_range + n * search_step
        stats = count_pairs(apply_time_offset(source, offset), target, max_dt)
        key = (stats[0], -stats[1], -stats[2])
        if best is None or key > best[0]:
            best = (key, offset, stats)
    return best[1], best[2]


def umeyama_se3(src_xyz, dst_xyz, use_z):
    src = src_xyz.copy()
    dst = dst_xyz.copy()
    if not use_z:
        src[:, 2] = 0.0
        dst[:, 2] = 0.0

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    h = src_centered.T @ dst_centered / src.shape[0]
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    if not use_z:
        yaw = math.atan2(r[1, 0], r[0, 0])
        c = math.cos(yaw)
        s = math.sin(yaw)
        r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    t = dst_mean - r @ src_mean
    return r, t


def transform_points(points, r, t):
    return (r @ points.T).T + t


def yaw_from_rot(r):
    return math.atan2(r[1, 0], r[0, 0])


def write_matrix(path, r, t):
    tf = np.eye(4)
    tf[:3, :3] = r
    tf[:3, 3] = t
    with open(path, "w", encoding="utf-8") as f:
        for row in tf:
            f.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Estimate rigid alignment from source trajectory to target trajectory."
    )
    parser.add_argument("--source", required=True, help="Source TUM trajectory, e.g. RTK/GNSS odom.")
    parser.add_argument("--target", required=True, help="Target TUM trajectory, e.g. Fast-LIVO2 odom.")
    parser.add_argument("--max-dt", type=float, default=0.05, help="Maximum timestamp difference for association.")
    parser.add_argument(
        "--time-offset",
        type=float,
        default=0.0,
        help="Seconds added to source timestamps before association.",
    )
    parser.add_argument(
        "--search-time-offset",
        type=float,
        default=0.0,
        help="Search +/- this many seconds for the best source timestamp offset.",
    )
    parser.add_argument(
        "--search-step",
        type=float,
        default=0.01,
        help="Time offset search step in seconds.",
    )
    parser.add_argument("--use-z", action="store_true", help="Estimate full 3D rotation instead of planar yaw alignment.")
    parser.add_argument("--out", default="", help="Optional output 4x4 matrix text file.")
    args = parser.parse_args()

    source = load_tum(Path(args.source).expanduser())
    target = load_tum(Path(args.target).expanduser())
    src_start, src_end = timestamp_range(source)
    dst_start, dst_end = timestamp_range(target)
    print(f"source_stamp_range: {src_start:.9f} -> {src_end:.9f} ({src_end - src_start:.3f} s)")
    print(f"target_stamp_range: {dst_start:.9f} -> {dst_end:.9f} ({dst_end - dst_start:.3f} s)")

    offset = args.time_offset
    if args.search_time_offset > 0.0:
        offset, stats = search_time_offset(
            source, target, args.max_dt, args.search_time_offset, args.search_step
        )
        print(
            f"best_time_offset: {offset:.6f} s "
            f"(matched {stats[0]}, mean_dt {stats[1]:.6f} s, max_dt {stats[2]:.6f} s)"
        )
    if offset != 0.0:
        source = apply_time_offset(source, offset)

    pairs = associate(source, target, args.max_dt)

    src_idx = [p[0] for p in pairs]
    dst_idx = [p[1] for p in pairs]
    dt = np.asarray([p[2] for p in pairs])
    src_xyz = source[src_idx, 1:4]
    dst_xyz = target[dst_idx, 1:4]

    r, t = umeyama_se3(src_xyz, dst_xyz, args.use_z)
    aligned = transform_points(src_xyz, r, t)
    residual = aligned - dst_xyz
    residual_xy = np.linalg.norm(residual[:, :2], axis=1)
    residual_3d = np.linalg.norm(residual, axis=1)

    print("Alignment source -> target")
    print(f"time_offset_applied: {offset:.6f} s")
    print(f"matched_pairs: {len(pairs)}")
    print(f"time_dt_mean: {dt.mean():.6f} s")
    print(f"time_dt_max: {dt.max():.6f} s")
    print(f"yaw_deg: {math.degrees(yaw_from_rot(r)):.6f}")
    print(f"translation_xyz: {t[0]:.6f} {t[1]:.6f} {t[2]:.6f}")
    print(f"rmse_xy: {math.sqrt(np.mean(residual_xy ** 2)):.6f} m")
    print(f"mean_xy: {residual_xy.mean():.6f} m")
    print(f"max_xy: {residual_xy.max():.6f} m")
    print(f"rmse_3d: {math.sqrt(np.mean(residual_3d ** 2)):.6f} m")
    print("T_source_target:")
    tf = np.eye(4)
    tf[:3, :3] = r
    tf[:3, 3] = t
    for row in tf:
        print(" ".join(f"{v:.9f}" for v in row))

    if args.out:
      out = Path(args.out).expanduser().resolve()
      out.parent.mkdir(parents=True, exist_ok=True)
      write_matrix(out, r, t)
      print(f"wrote: {out}")


if __name__ == "__main__":
    main()
