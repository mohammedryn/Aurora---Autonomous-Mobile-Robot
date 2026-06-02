#!/usr/bin/env python3
"""
map_cleaner.py — post-process a ROS2 .pgm occupancy map to remove noise.

Usage:
    python3 tools/map_cleaner.py ~/AMR/maps/explore_map.pgm

Writes:  <input>_clean.pgm  (drop-in replacement — same .yaml still works)

How it works:
  ROS2 map pixels:  0 = occupied (black)  |  205 = unknown (grey)  |  254 = free (white)
  1. Find all connected clusters of occupied pixels.
  2. Drop clusters smaller than --min-size pixels (noise/scatter).
  3. Optionally close small gaps in walls with --close-walls.

Requirements:
    sudo apt install python3-scipy python3-pil   (usually already present)
"""
import sys
import argparse
import numpy as np
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    sys.exit("Pillow not found: sudo apt install python3-pil")

try:
    from scipy import ndimage
except ImportError:
    sys.exit("scipy not found: sudo apt install python3-scipy")


def clean_map(pgm_path: Path, min_size: int, close_iters: int, verbose: bool) -> Path:
    img = np.array(Image.open(pgm_path))

    # occupied = black (value < 50 to be safe with compression artefacts)
    occupied = img < 50

    # --- label connected occupied clusters ---
    struct = ndimage.generate_binary_structure(2, 2)  # 8-connectivity
    labeled, n_clusters = ndimage.label(occupied, structure=struct)
    sizes = ndimage.sum(occupied, labeled, range(1, n_clusters + 1))

    if verbose:
        print(f"  Total occupied clusters : {n_clusters}")
        print(f"  Clusters < {min_size} px (noise) : {sum(1 for s in sizes if s < min_size)}")
        print(f"  Clusters >= {min_size} px (kept)  : {sum(1 for s in sizes if s >= min_size)}")

    # build mask of clusters to KEEP
    keep_labels = np.array([i + 1 for i, s in enumerate(sizes) if s >= min_size])
    if len(keep_labels) == 0:
        print("  WARNING: no clusters survived — check --min-size value")
        return pgm_path

    keep_mask = np.isin(labeled, keep_labels)

    # --- optional: close small gaps in walls (connects broken wall segments) ---
    if close_iters > 0:
        close_struct = ndimage.generate_binary_structure(2, 1)  # 4-connectivity
        keep_mask = ndimage.binary_closing(
            keep_mask, structure=close_struct, iterations=close_iters)

    # --- rebuild image ---
    result = img.copy()
    # pixels that were occupied but got removed → mark as free
    result[occupied & ~keep_mask] = 254
    # pixels added by closing → mark as occupied
    result[~occupied & keep_mask] = 0

    out_path = pgm_path.parent / (pgm_path.stem + "_clean.pgm")
    Image.fromarray(result).save(out_path)
    return out_path


def main():
    parser = argparse.ArgumentParser(description="Clean ROS2 .pgm map noise")
    parser.add_argument("pgm", help="Input .pgm map file")
    parser.add_argument("--min-size", type=int, default=10,
                        help="Min cluster size to keep (pixels). Default: 10. "
                             "Increase to remove larger blobs.")
    parser.add_argument("--close-walls", type=int, default=1,
                        help="Morphological closing iterations to fill wall gaps. "
                             "Default: 1. Set 0 to disable.")
    parser.add_argument("--verbose", action="store_true", default=True)
    args = parser.parse_args()

    pgm_path = Path(args.pgm).expanduser().resolve()
    if not pgm_path.exists():
        sys.exit(f"File not found: {pgm_path}")
    if pgm_path.suffix.lower() not in (".pgm", ".png"):
        sys.exit("Expected a .pgm file")

    print(f"Input : {pgm_path}")
    out = clean_map(pgm_path, args.min_size, args.close_walls, args.verbose)
    print(f"Output: {out}")
    print("Done. Use the _clean.pgm with the same .yaml file for navigation.")


if __name__ == "__main__":
    main()
