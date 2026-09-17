"""
Landing-board geometry: tag36h11 multi-scale board, per the senior's thesis Table 3.2
(see docs/01-inherited-system.md) and CLAUDE.md sections 3/8.

60cm board. One 24cm tag (ID 0) at centre, four 8cm tags (IDs 1-4) at the corners,
offset +-0.22m from centre in the board plane. This is the single source of truth
for board geometry -- the synthetic camera projects it, the detector's object points
are built from it, and fusion's per-tag weighting keys off MARKER_SIZES/MARKER_OFFSETS.

Board-plane convention: +X = right when looking down at the board from above,
+Y = forward (matches CLAUDE.md's body-frame convention so no extra flip is needed
when converting a fused board-plane position into body_x/body_y later).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Tag:
    id: int
    size_m: float          # physical size of the black square, metres
    offset_x_m: float       # offset of this tag's centre from the board/landing centre
    offset_y_m: float


CENTER_TAG_SIZE_M = 0.24
CORNER_TAG_SIZE_M = 0.08
CORNER_OFFSET_M = 0.22

TAGS: dict[int, Tag] = {
    0: Tag(id=0, size_m=CENTER_TAG_SIZE_M, offset_x_m=0.00, offset_y_m=0.00),
    1: Tag(id=1, size_m=CORNER_TAG_SIZE_M, offset_x_m=-CORNER_OFFSET_M, offset_y_m=+CORNER_OFFSET_M),
    2: Tag(id=2, size_m=CORNER_TAG_SIZE_M, offset_x_m=+CORNER_OFFSET_M, offset_y_m=+CORNER_OFFSET_M),
    3: Tag(id=3, size_m=CORNER_TAG_SIZE_M, offset_x_m=-CORNER_OFFSET_M, offset_y_m=-CORNER_OFFSET_M),
    4: Tag(id=4, size_m=CORNER_TAG_SIZE_M, offset_x_m=+CORNER_OFFSET_M, offset_y_m=-CORNER_OFFSET_M),
}

TAG_FAMILY = "DICT_APRILTAG_36h11"


def tag_corners_board_frame(tag: Tag):
    """
    Return this tag's 4 corners in the BOARD frame (metres, Z=0 plane), in the order
    cv2.aruco's ArucoDetector actually returns detected corners: top-left, top-right,
    bottom-right, bottom-left, going clockwise starting from top-left as seen in the
    image -- NOT the official upstream AprilTag library's own corner ordering.
    Confirmed divergence: https://github.com/opencv/opencv-python/issues/1195

    "Top" here means +Y in the board frame (since +Y is defined as "forward"/up when
    looking down at the board with a fixed, un-rotated tag). src/detector.py's object
    points MUST be built with this exact ordering, and the corner-order sanity test
    (first thing built against detector.py) exists specifically to catch a mismatch
    here before it produces a silently-wrong pose.
    """
    half = tag.size_m / 2.0
    cx, cy = tag.offset_x_m, tag.offset_y_m
    return [
        (cx - half, cy + half, 0.0),   # top-left
        (cx + half, cy + half, 0.0),   # top-right
        (cx + half, cy - half, 0.0),   # bottom-right
        (cx - half, cy - half, 0.0),   # bottom-left
    ]
