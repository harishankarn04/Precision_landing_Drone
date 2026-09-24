#!/usr/bin/env python3
"""
One-off asset generator: renders the board texture (sim/board.py's render_board_texture)
and saves it as materials/textures/board.png for model.sdf's PBR albedo_map.

Re-run this if sim/board.py's tag geometry ever changes -- the PNG is a generated
artifact, not something to hand-edit.

    .venv/bin/python sim/gazebo_models/precision_landing_board/generate_texture.py
"""
import os
import sys

import cv2

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from sim.board import render_board_texture

if __name__ == "__main__":
    texture = render_board_texture(px_per_meter=1000)
    out_path = os.path.join(os.path.dirname(__file__), "materials", "textures", "board.png")
    cv2.imwrite(out_path, texture)
    print(f"Wrote {out_path} ({texture.shape[1]}x{texture.shape[0]}px)")
