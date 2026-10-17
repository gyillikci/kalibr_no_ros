#!/usr/bin/env python3
"""Render a Kalibr AprilGrid seen by a known pinhole-radtan camera into a bag.

The target is drawn exactly as kalibr_create_target_pdf draws it (36h11 codes,
two-bit black border, rotation 2, corner squares) and the grid geometry follows
GridCalibrationTargetAprilgrid::createGridPoints. Because the true intrinsics
are known, the bag is an end-to-end check that does not depend on downloading
a dataset: a correct build must recover the values written to --truth.
"""

import argparse
import math
import re
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bag_io" / "src"))

from kalibr_bag_io import BagWriter, ImageRecord  # noqa: E402

TARGET_PDF_SCRIPT = (
    ROOT / "upstream" / "kalibr" / "aslam_offline_calibration" / "kalibr"
    / "python" / "kalibr_create_target_pdf"
)

# EuRoC cam0, as published with the dataset.
DEFAULT_CAMERA = {
    "resolution": [752, 480],
    "intrinsics": [458.654, 457.296, 367.215, 248.375],
    "distortion_coeffs": [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05],
}


def tag36h11_codes():
    text = TARGET_PDF_SCRIPT.read_text()
    match = re.search(r"t36h11\s*=\s*\[([^\]]*)\]", text)
    if match is None:
        raise RuntimeError("t36h11 codes not found in {}".format(TARGET_PDF_SCRIPT))
    return [int(code, 16) for code in re.findall(r"0x[0-9a-fA-F]+", match.group(1))]


def render_board(rows, cols, tag_size, spacing, pixels_per_metre, margin):
    """Return the board texture and the matrix mapping texture pixels to metres."""
    codes = tag36h11_codes()
    border_bits, data_bits = 2, 6
    bit = tag_size / (data_bits + 2 * border_bits)
    square = spacing * tag_size
    width = cols * tag_size + (cols - 1) * square + 2 * margin
    height = rows * tag_size + (rows - 1) * square + 2 * margin
    w_px = int(math.ceil(width * pixels_per_metre))
    h_px = int(math.ceil(height * pixels_per_metre))
    board = np.full((h_px, w_px), 255, np.uint8)

    def fill(x, y, w, h):
        # Rectangle in the y-up target frame, as PyX draws it.
        u0 = int(round((x + margin) * pixels_per_metre))
        u1 = int(round((x + w + margin) * pixels_per_metre))
        v0 = int(round(h_px - (y + h + margin) * pixels_per_metre))
        v1 = int(round(h_px - (y + margin) * pixels_per_metre))
        board[max(v0, 0):max(v1, 0), max(u0, 0):max(u1, 0)] = 0

    for ty in range(rows):
        for tx in range(cols):
            tag_id = cols * ty + tx
            x0 = tx * (1 + spacing) * tag_size
            y0 = ty * (1 + spacing) * tag_size
            fill(x0, y0, tag_size, tag_size)
            # Clear the data area, then draw the black data bits.
            u0 = int(round((x0 + border_bits * bit + margin) * pixels_per_metre))
            u1 = int(round((x0 + tag_size - border_bits * bit + margin) * pixels_per_metre))
            v0 = int(round(h_px - (y0 + tag_size - border_bits * bit + margin) * pixels_per_metre))
            v1 = int(round(h_px - (y0 + border_bits * bit + margin) * pixels_per_metre))
            board[v0:v1, u0:u1] = 255
            code = codes[tag_id]
            matrix = np.zeros((data_bits, data_bits))
            for i in range(data_bits):
                for j in range(data_bits):
                    if not code & (1 << data_bits * i + j):
                        matrix[i, j] = 1
            matrix = np.rot90(matrix, 2)
            for i in range(data_bits):
                for j in range(data_bits):
                    if matrix[i, j]:
                        fill(x0 + (j + border_bits) * bit,
                             y0 + ((border_bits - 1) + data_bits - i) * bit,
                             bit, bit)
            for cx, cy in ((x0 - square, y0 - square), (x0 + tag_size, y0 - square),
                           (x0 + tag_size, y0 + tag_size), (x0 - square, y0 + tag_size)):
                fill(cx, cy, square, square)

    texture_to_metric = np.array([
        [1.0 / pixels_per_metre, 0.0, -margin],
        [0.0, -1.0 / pixels_per_metre, h_px / pixels_per_metre - margin],
        [0.0, 0.0, 1.0],
    ])
    return board, texture_to_metric


def rotation(rx, ry, rz):
    return cv2.Rodrigues(np.array([rx, ry, rz], dtype=float))[0]


def sample_poses(count, board_centre, rng):
    """Target-to-camera poses that keep the board in front of the camera."""
    poses = []
    for _ in range(count):
        # Tilt the board about its own centre, then place that centre in view.
        R = rotation(math.pi + rng.uniform(-0.6, 0.6),
                     rng.uniform(-0.6, 0.6), rng.uniform(-0.4, 0.4))
        depth = rng.uniform(0.7, 1.4)
        centre_cam = np.array([rng.uniform(-0.18, 0.18) * depth,
                               rng.uniform(-0.12, 0.12) * depth, depth])
        t = centre_cam - R @ board_centre
        poses.append((R, t))
    return poses


def distortion_maps(K, D, width, height, scale):
    """For each distorted output pixel, its location in the ideal image."""
    u, v = np.meshgrid(np.arange(width, dtype=np.float64),
                       np.arange(height, dtype=np.float64))
    pts = np.stack([u.ravel(), v.ravel()], axis=1).reshape(-1, 1, 2)
    ideal = cv2.undistortPointsIter(
        pts, K, D, None, K,
        (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-10),
    ).reshape(height, width, 2)
    map_x = (ideal[..., 0] * scale + (scale - 1) / 2).astype(np.float32)
    map_y = (ideal[..., 1] * scale + (scale - 1) / 2).astype(np.float32)
    return map_x, map_y


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="ROS1 .bag file or ROS2 bag directory")
    parser.add_argument("--target", required=True, help="AprilGrid YAML to write")
    parser.add_argument("--truth", required=True, help="ground-truth camera YAML to write")
    parser.add_argument("--topic", default="/cam0/image_raw")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--rate", type=float, default=4.0, help="frames per second")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--noise", type=float, default=1.5, help="pixel noise sigma")
    args = parser.parse_args()

    rows = cols = 6
    tag_size, spacing = 0.088, 0.3
    width, height = DEFAULT_CAMERA["resolution"]
    fu, fv, cu, cv = DEFAULT_CAMERA["intrinsics"]
    K = np.array([[fu, 0, cu], [0, fv, cv], [0, 0, 1]], dtype=float)
    D = np.array(DEFAULT_CAMERA["distortion_coeffs"], dtype=float)

    board, texture_to_metric = render_board(rows, cols, tag_size, spacing,
                                            pixels_per_metre=4000, margin=0.05)
    extent = cols * tag_size + (cols - 1) * spacing * tag_size
    board_centre = np.array([extent / 2, extent / 2, 0.0])

    scale = 3  # supersampling factor for the ideal pinhole image
    K_ss = K.copy()
    K_ss[:2] *= scale
    K_ss[0, 2] = K[0, 2] * scale + (scale - 1) / 2
    K_ss[1, 2] = K[1, 2] * scale + (scale - 1) / 2
    map_x, map_y = distortion_maps(K, D, width, height, scale)

    rng = np.random.default_rng(args.seed)
    poses = sample_poses(args.frames, board_centre, rng)
    period_ns = int(round(1e9 / args.rate))
    start_ns = 1_403_715_000_000_000_000

    with BagWriter(args.output) as writer:
        for index, (R, t) in enumerate(poses):
            H = K_ss @ np.column_stack([R[:, 0], R[:, 1], t]) @ texture_to_metric
            ideal = cv2.warpPerspective(
                board, H, (width * scale, height * scale),
                flags=cv2.INTER_AREA, borderValue=160)
            ideal = cv2.GaussianBlur(ideal, (0, 0), 0.5 * scale)
            image = cv2.remap(ideal, map_x, map_y, cv2.INTER_LINEAR,
                              borderValue=160).astype(np.float64)
            image += rng.normal(0.0, args.noise, image.shape)
            image = np.clip(np.rint(image), 0, 255).astype(np.uint8)
            stamp = start_ns + index * period_ns
            writer.write_image(args.topic, ImageRecord(
                header_timestamp_ns=stamp, record_timestamp_ns=stamp,
                encoding="mono8", image=image, frame_id="cam0", sequence=index))

    Path(args.target).write_text(yaml.safe_dump({
        "target_type": "aprilgrid", "tagCols": cols, "tagRows": rows,
        "tagSize": tag_size, "tagSpacing": spacing,
    }, sort_keys=False))
    Path(args.truth).write_text(yaml.safe_dump({
        "camera_model": "pinhole", "distortion_model": "radtan",
        **DEFAULT_CAMERA, "rostopic": args.topic,
    }, sort_keys=False))
    print("wrote {} frames to {}".format(len(poses), args.output))


if __name__ == "__main__":
    main()
