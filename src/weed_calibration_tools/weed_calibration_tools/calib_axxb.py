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
    if len(rows) < 4:
        raise RuntimeError(f"Need at least 4 poses in {path}")
    return np.asarray(rows, dtype=np.float64)


def quat_to_rot(qx, qy, qz, qw):
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    q /= n
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_to_matrix(row):
    t = row[1:4]
    r = quat_to_rot(row[4], row[5], row[6], row[7])
    tf = np.eye(4)
    tf[:3, :3] = r
    tf[:3, 3] = t
    return tf


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
    if len(pairs) < 4:
        raise RuntimeError(f"Only matched {len(pairs)} pose pairs; increase --max-dt or check timestamps")
    return pairs


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
    best = None
    steps = int(round((2.0 * search_range) / search_step))
    for n in range(steps + 1):
        offset = -search_range + n * search_step
        stats = count_pairs(apply_time_offset(source, offset), target, max_dt)
        key = (stats[0], -stats[1], -stats[2])
        if best is None or key > best[0]:
            best = (key, offset, stats)
    return best[1], best[2]


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rot_to_quat_xyzw(r):
    tr = np.trace(r)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    return q / np.linalg.norm(q)


def quat_left(q):
    x, y, z, w = q
    v = np.array([x, y, z])
    m = np.zeros((4, 4))
    m[:3, :3] = w * np.eye(3) + skew(v)
    m[:3, 3] = v
    m[3, :3] = -v
    m[3, 3] = w
    return m


def quat_right(q):
    x, y, z, w = q
    v = np.array([x, y, z])
    m = np.zeros((4, 4))
    m[:3, :3] = w * np.eye(3) - skew(v)
    m[:3, 3] = v
    m[3, :3] = -v
    m[3, 3] = w
    return m


def motion_pairs(poses_a, poses_b, skip, min_translation, min_rotation_deg):
    motions = []
    min_rot = math.radians(min_rotation_deg)
    for i in range(0, len(poses_a) - skip, skip):
        a = np.linalg.inv(poses_a[i]) @ poses_a[i + skip]
        b = np.linalg.inv(poses_b[i]) @ poses_b[i + skip]
        dist = max(np.linalg.norm(a[:3, 3]), np.linalg.norm(b[:3, 3]))
        angle = max(rotation_angle(a[:3, :3]), rotation_angle(b[:3, :3]))
        if dist >= min_translation or angle >= min_rot:
            motions.append((a, b))
    if len(motions) < 3:
        raise RuntimeError("Too few useful motion pairs; reduce --skip or thresholds, or use a bag with richer motion")
    return motions


def rotation_angle(r):
    c = (np.trace(r) - 1.0) * 0.5
    return math.acos(float(np.clip(c, -1.0, 1.0)))


def solve_rotation(motions):
    rows = []
    for a, b in motions:
        qa = rot_to_quat_xyzw(a[:3, :3])
        qb = rot_to_quat_xyzw(b[:3, :3])
        if qa[3] < 0.0:
            qa *= -1.0
        if qb[3] < 0.0:
            qb *= -1.0
        rows.append(quat_left(qa) - quat_right(qb))
    m = np.vstack(rows)
    _, _, vt = np.linalg.svd(m)
    qx = vt[-1]
    qx /= np.linalg.norm(qx)
    return quat_to_rot(qx[0], qx[1], qx[2], qx[3])


def solve_translation(motions, rx):
    lhs = []
    rhs = []
    for a, b in motions:
        ra = a[:3, :3]
        ta = a[:3, 3]
        tb = b[:3, 3]
        lhs.append(ra - np.eye(3))
        rhs.append(rx @ tb - ta)
    lhs = np.vstack(lhs)
    rhs = np.concatenate(rhs)
    tx, *_ = np.linalg.lstsq(lhs, rhs, rcond=None)
    return tx


def evaluate(motions, x):
    rot_errors = []
    trans_errors = []
    x_inv = np.linalg.inv(x)
    for a, b in motions:
        err = np.linalg.inv(a @ x) @ (x @ b)
        rot_errors.append(math.degrees(rotation_angle(err[:3, :3])))
        trans_errors.append(np.linalg.norm(err[:3, 3]))
    return np.asarray(rot_errors), np.asarray(trans_errors)


def write_matrix(path, tf):
    with open(path, "w", encoding="utf-8") as f:
        for row in tf:
            f.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Calibrate X in A X = X B from two TUM trajectories.")
    parser.add_argument("--traj-a", required=True, help="Trajectory A TUM file, e.g. RTK/INS trajectory.")
    parser.add_argument("--traj-b", required=True, help="Trajectory B TUM file, e.g. Fast-LIVO2 trajectory.")
    parser.add_argument("--max-dt", type=float, default=0.05)
    parser.add_argument("--time-offset", type=float, default=0.0, help="Seconds added to trajectory A timestamps.")
    parser.add_argument("--search-time-offset", type=float, default=0.0, help="Search +/- this many seconds for A timestamp offset.")
    parser.add_argument("--search-step", type=float, default=0.01, help="Time offset search step in seconds.")
    parser.add_argument("--skip", type=int, default=10, help="Pose interval used to build relative motions.")
    parser.add_argument("--min-translation", type=float, default=0.2)
    parser.add_argument("--min-rotation-deg", type=float, default=1.0)
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    traj_a = load_tum(Path(args.traj_a).expanduser())
    traj_b = load_tum(Path(args.traj_b).expanduser())
    offset = args.time_offset
    if args.search_time_offset > 0.0:
        offset, stats = search_time_offset(
            traj_a, traj_b, args.max_dt, args.search_time_offset, args.search_step
        )
        print(
            f"best_time_offset: {offset:.6f} s "
            f"(matched {stats[0]}, mean_dt {stats[1]:.6f} s, max_dt {stats[2]:.6f} s)"
        )
    if offset != 0.0:
        traj_a = apply_time_offset(traj_a, offset)
    pairs = associate(traj_a, traj_b, args.max_dt)
    dt = np.asarray([p[2] for p in pairs])
    poses_a = [pose_to_matrix(traj_a[i]) for i, _, _ in pairs]
    poses_b = [pose_to_matrix(traj_b[j]) for _, j, _ in pairs]
    motions = motion_pairs(poses_a, poses_b, args.skip, args.min_translation, args.min_rotation_deg)

    rx = solve_rotation(motions)
    tx = solve_translation(motions, rx)
    x = np.eye(4)
    x[:3, :3] = rx
    x[:3, 3] = tx
    rot_err, trans_err = evaluate(motions, x)

    print("AX=XB calibration, A X = X B")
    print(f"time_offset_applied: {offset:.6f} s")
    print(f"matched_poses: {len(pairs)}")
    print(f"motion_pairs: {len(motions)}")
    print(f"time_dt_mean: {dt.mean():.6f} s")
    print(f"time_dt_max: {dt.max():.6f} s")
    print(f"rotation_error_mean_deg: {rot_err.mean():.6f}")
    print(f"rotation_error_max_deg: {rot_err.max():.6f}")
    print(f"translation_error_mean_m: {trans_err.mean():.6f}")
    print(f"translation_error_max_m: {trans_err.max():.6f}")
    print("T_A_B:")
    for row in x:
        print(" ".join(f"{v:.9f}" for v in row))
    if args.out:
        out = Path(args.out).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        write_matrix(out, x)
        print(f"wrote: {out}")


if __name__ == "__main__":
    main()
