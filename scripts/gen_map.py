#!/usr/bin/env python3
"""Generate a Nav2 occupancy map from the maze layout.

The map is derived from the *known* layout so that AMCL/Nav2 have a reliable
ground-truth map even before a SLAM run. During the project you still produce a
map with SLAM Toolbox and can replace these files with the SLAM result.

Run:  python3 scripts/gen_map.py
Out:  maze_ws/src/maze_navigation/maps/maze.pgm + maze.yaml
"""
import os

# Must match scripts/gen_maze.py
LAYOUT = [
    "11111111",
    "10000001",
    "10111101",
    "10010001",
    "10100101",
    "10111101",
    "10000001",
    "11111111",
]

CELL = 0.5
RESOLUTION = 0.05
ROWS = len(LAYOUT)
COLS = len(LAYOUT[0])

WIDTH = int(round((COLS * CELL) / RESOLUTION))
HEIGHT = int(round((ROWS * CELL) / RESOLUTION))
ORIGIN_X = -(COLS * CELL) / 2.0
ORIGIN_Y = -(ROWS * CELL) / 2.0

OCCUPIED = 0      # black in pgm
FREE = 254        # white in pgm


def build_pixels():
    rows = []
    for py in range(HEIGHT):        # py = 0 is the TOP row of the image (max y)
        row = []
        for px in range(WIDTH):
            wx = ORIGIN_X + (px + 0.5) * RESOLUTION
            wy = ORIGIN_Y + (HEIGHT - py - 0.5) * RESOLUTION
            col = int((wx - ORIGIN_X) // CELL)
            # row index in LAYOUT measured from the top
            r = int((ORIGIN_Y + ROWS * CELL - wy) // CELL)
            col = max(0, min(COLS - 1, col))
            r = max(0, min(ROWS - 1, r))
            row.append(OCCUPIED if LAYOUT[r][col] == "1" else FREE)
        rows.append(row)
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.normpath(
        os.path.join(here, "..", "maze_ws", "src", "maze_navigation", "maps")
    )
    os.makedirs(out_dir, exist_ok=True)

    pixels = build_pixels()

    pgm_path = os.path.join(out_dir, "maze.pgm")
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n")
        f.write(b"# maze occupancy map\n")
        f.write(f"{WIDTH} {HEIGHT}\n".encode())
        f.write(b"255\n")
        for row in pixels:
            f.write(bytes(row))

    yaml_path = os.path.join(out_dir, "maze.yaml")
    with open(yaml_path, "w") as f:
        f.write("image: maze.pgm\n")
        f.write(f"resolution: {RESOLUTION}\n")
        f.write(f"origin: [{ORIGIN_X:.3f}, {ORIGIN_Y:.3f}, 0.000]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")

    print(f"Wrote {pgm_path} ({WIDTH}x{HEIGHT})")
    print(f"Wrote {yaml_path}")


if __name__ == "__main__":
    main()
