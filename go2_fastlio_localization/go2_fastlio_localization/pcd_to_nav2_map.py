#!/usr/bin/env python3
"""Project a binary/ascii PCD map into a Nav2 occupancy map."""

import argparse
import math
import os
import struct

import numpy as np


def read_pcd_xyz(path):
    header = {}
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA line")
            text = line.decode("ascii", errors="strict").strip()
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break

        fields = header["FIELDS"]
        sizes = [int(v) for v in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
        points = int(header.get("POINTS", header["WIDTH"])[0])
        mode = header["DATA"][0].lower()

        if any(count != 1 for count in counts):
            raise ValueError("Only scalar PCD fields are supported")

        type_map = {
            ("F", 4): "<f4", ("F", 8): "<f8",
            ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4",
            ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4",
        }
        dtype = np.dtype([
            (name, type_map[(kind.upper(), size)])
            for name, kind, size in zip(fields, types, sizes)
        ])

        if mode == "binary":
            records = np.fromfile(stream, dtype=dtype, count=points)
            xyz = np.column_stack((records["x"], records["y"], records["z"]))
        elif mode == "ascii":
            values = np.loadtxt(stream, dtype=np.float64, max_rows=points)
            indices = [fields.index(axis) for axis in ("x", "y", "z")]
            xyz = values[:, indices]
        else:
            raise ValueError(f"Unsupported PCD DATA mode: {mode}")

    return np.asarray(xyz, dtype=np.float64)


def dilate(mask, iterations):
    result = mask.copy()
    for _ in range(iterations):
        padded = np.pad(result, 1, mode="constant", constant_values=False)
        expanded = np.zeros_like(result)
        for row_shift in range(3):
            for col_shift in range(3):
                expanded |= padded[
                    row_shift:row_shift + result.shape[0],
                    col_shift:col_shift + result.shape[1],
                ]
        result = expanded
    return result


def write_map(xyz, output_prefix, resolution, z_min, z_max, padding,
              min_points, dilation_cells, *, x_min=None, x_max=None,
              y_min=None, y_max=None):
    """Optional crop uses PCD coordinates and half-open [min, max) bounds.

    Supply all four bounds or none. Explicit bounds ignore padding; the last
    cell may extend beyond max when the span is not divisible by resolution.
    The original full-map extent calculation is retained without bounds.
    """
    bounds = (x_min, x_max, y_min, y_max)
    cropped = any(value is not None for value in bounds)
    if cropped:
        if any(value is None for value in bounds):
            raise ValueError("Crop requires --x-min, --x-max, --y-min and --y-max together")
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError("Crop bounds must be finite")
        if x_min >= x_max or y_min >= y_max:
            raise ValueError("Crop minimum must be smaller than maximum on each axis")
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("Resolution must be finite and positive")

    finite = np.isfinite(xyz).all(axis=1)
    if cropped:
        finite &= (
            (xyz[:, 0] >= x_min) & (xyz[:, 0] < x_max) &
            (xyz[:, 1] >= y_min) & (xyz[:, 1] < y_max)
        )
    xy_extent = xyz[finite, :2]
    obstacle_points = xyz[
        finite & (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max), :2
    ]
    if len(obstacle_points) == 0:
        raise ValueError("No points remain after the crop and height filters" if cropped
                         else "No points remain after the height filter")

    if cropped:
        min_x, min_y = x_min, y_min
        width = math.ceil((x_max - x_min) / resolution)
        height = math.ceil((y_max - y_min) / resolution)
    else:
        min_x = math.floor((xy_extent[:, 0].min() - padding) / resolution) * resolution
        min_y = math.floor((xy_extent[:, 1].min() - padding) / resolution) * resolution
        max_x = math.ceil((xy_extent[:, 0].max() + padding) / resolution) * resolution
        max_y = math.ceil((xy_extent[:, 1].max() + padding) / resolution) * resolution
        width = int(round((max_x - min_x) / resolution)) + 1
        height = int(round((max_y - min_y) / resolution)) + 1

    cols = np.floor((obstacle_points[:, 0] - min_x) / resolution).astype(int)
    rows = np.floor((obstacle_points[:, 1] - min_y) / resolution).astype(int)
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    counts = np.zeros((height, width), dtype=np.uint16)
    np.add.at(counts, (rows[valid], cols[valid]), 1)
    occupied = dilate(counts >= min_points, dilation_cells)

    # PGM is written top-to-bottom while OccupancyGrid row zero starts at min_y.
    image = np.full((height, width), 254, dtype=np.uint8)
    image[occupied] = 0
    image = np.flipud(image)

    pgm_path = output_prefix + ".pgm"
    yaml_path = output_prefix + ".yaml"
    os.makedirs(os.path.dirname(os.path.abspath(pgm_path)), exist_ok=True)
    with open(pgm_path, "wb") as stream:
        stream.write(f"P5\n# projected from 3D PCD\n{width} {height}\n255\n".encode())
        stream.write(image.tobytes())
    with open(yaml_path, "w", encoding="utf-8") as stream:
        stream.write(
            f"image: {os.path.basename(pgm_path)}\n"
            f"resolution: {resolution:.6f}\n"
            f"origin: [{min_x:.6f}, {min_y:.6f}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n"
        )

    print(f"Input points: {len(xyz)}")
    print(f"Obstacle points: {len(obstacle_points)} (z {z_min} to {z_max} m)")
    print(f"Map: {width} x {height}, resolution {resolution} m")
    print(f"Origin: [{min_x}, {min_y}, 0.0]")
    print(f"Occupied cells after dilation: {int(occupied.sum())}")
    print(f"Wrote {pgm_path}")
    print(f"Wrote {yaml_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd")
    parser.add_argument("output_prefix")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--z-min", type=float, default=-0.15)
    parser.add_argument("--z-max", type=float, default=1.50)
    parser.add_argument("--padding", type=float, default=0.30,
                        help="Full-map margin in meters; ignored with explicit crop bounds")
    crop = parser.add_argument_group(
        "optional XY crop", "Supply all four bounds in PCD coordinates (meters); "
        "lower bounds inclusive, upper bounds exclusive. Omit all for original full-map behavior.")
    crop.add_argument("--x-min", type=float, default=None)
    crop.add_argument("--x-max", type=float, default=None)
    crop.add_argument("--y-min", type=float, default=None)
    crop.add_argument("--y-max", type=float, default=None)
    parser.add_argument("--min-points", type=int, default=2)
    parser.add_argument("--dilation-cells", type=int, default=1)
    args = parser.parse_args()
    try:
        write_map(
            read_pcd_xyz(args.pcd), args.output_prefix, args.resolution,
            args.z_min, args.z_max, args.padding, args.min_points,
            args.dilation_cells,
            x_min=args.x_min, x_max=args.x_max,
            y_min=args.y_min, y_max=args.y_max,
        )
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
