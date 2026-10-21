#!/usr/bin/env python3
"""Draw the sensor_config icon as SVG and PNGs next to this script."""

# A sensor slab in isometric view with its X / Y / Z axes and the level ring,
# on the Timing board tile so it sits beside Nereus. Generic on purpose: no
# vendor name, mark or case outline. Small sizes zoom in with heavier strokes.
#
# Regenerate: uv run --with cairosvg python riptide_imu/icons/make_icons.py

import math
from pathlib import Path

import cairosvg


HERE = Path(__file__).resolve().parent
SIZES = (16, 24, 32, 48, 64, 128, 256, 512)

# Timing board colors, and the app's axis colors lifted for a dark tile.
TILE_TOP, TILE_BOTTOM, TILE_EDGE = "#262b32", "#0c0e11", "#3a414b"
TOP, LEFT, RIGHT, EDGE = "#e9ecef", "#98a1ac", "#6b7480", "#0c0e11"
RING = "#6ea2ff"
AXES = {"x": "#ff6b6b", "y": "#3ddc97", "z": "#5aa2ff"}

# Sensor frame on screen: X up-left, Y up-right, Z down, as on the VN-100.
X = (-math.cos(math.pi / 6), -0.5)
Y = (math.cos(math.pi / 6), -0.5)
Z = (0.0, 1.0)
ORIGIN = (256.0, 246.0)


def point(x, y, z):
    """Project sensor-frame coordinates, in pixels, to the screen."""
    return (ORIGIN[0] + x * X[0] + y * Y[0] + z * Z[0],
            ORIGIN[1] + x * X[1] + y * Y[1] + z * Z[1])


def polygon(points, fill, stroke=None, width=0):
    """Return an SVG polygon, outlined when stroke is given."""
    text = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    outline = ""
    if stroke:
        outline = (f' stroke="{stroke}" stroke-width="{width}"'
                   ' stroke-linejoin="round"')
    return f'  <polygon points="{text}" fill="{fill}"{outline}/>'


def arrow(start, end, color, width):
    """Return a round-capped shaft with a triangular head at end."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    head = width * 2.3
    neck = (end[0] - ux * head, end[1] - uy * head)
    left = (neck[0] - uy * head * 0.62, neck[1] + ux * head * 0.62)
    right = (neck[0] + uy * head * 0.62, neck[1] - ux * head * 0.62)
    shaft = (f'  <path d="M{start[0]:.1f} {start[1]:.1f} '
             f'L{neck[0]:.1f} {neck[1]:.1f}" stroke="{color}" '
             f'stroke-width="{width}" stroke-linecap="round" fill="none"/>')
    return shaft + "\n" + polygon((end, left, right), color)


def ring(radius, z, front, color, width, opacity):
    """Return the near (front) or far half of the level ring."""
    steps = 48
    # The near half is the one drawn over the slab.
    start = math.radians(45 if front else 225)
    points = []
    for i in range(steps + 1):
        angle = start - math.pi * i / steps
        points.append(point(radius * math.cos(angle),
                            radius * math.sin(angle), z))
    path = " ".join(f"{'M' if i == 0 else 'L'}{x:.1f} {y:.1f}"
                    for i, (x, y) in enumerate(points))
    return (f'  <path d="{path}" fill="none" stroke="{color}" '
            f'stroke-width="{width}" stroke-linecap="round" '
            f'opacity="{opacity}"/>')


def icon(small=False):
    """Return the whole tile; small zooms in for 16-48 px."""
    # Slab half-length, half-width and thickness, in VN-100 proportions.
    a, b, t = 100, 93, 36
    axis_width, ring_width = (36, 22) if small else (20, 11)
    z_end = 404 if small else 438
    edge = (EDGE, 5) if not small else (None, 0)
    top_face = [point(a, b, -t), point(a, -b, -t),
                point(-a, -b, -t), point(-a, b, -t)]
    minus_x = [point(-a, -b, -t), point(-a, b, -t),
               point(-a, b, 0), point(-a, -b, 0)]
    minus_y = [point(-a, -b, -t), point(a, -b, -t),
               point(a, -b, 0), point(-a, -b, 0)]
    center = point(0, 0, -t)
    # Z leaves the bottom face; only the part below the slab shows.
    z_from = (ORIGIN[0], point(-a, -b, 0)[1] + 4)
    border = ""
    if not small:
        border = ('  <rect x="16" y="16" width="480" height="480" rx="112" '
                  f'fill="none" stroke="{TILE_EDGE}" stroke-width="3"/>')
    zoom = ""
    if small:
        zoom = ('  <g transform="translate(256 262) scale(1.2) '
                'translate(-256 -250)">')
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" '
        'width="512" height="512">',
        "  <title>VectorNav Workshop</title>",
        "  <defs>",
        '    <linearGradient id="bg" x1="0" y1="0" x2="0" y2="1">',
        f'      <stop offset="0" stop-color="{TILE_TOP}"/>'
        f'<stop offset="1" stop-color="{TILE_BOTTOM}"/>',
        "    </linearGradient>",
        "  </defs>",
        '  <rect x="16" y="16" width="480" height="480" rx="112" '
        'fill="url(#bg)"/>',
        border,
        zoom,
        ring(152, -t / 2, False, RING, ring_width, 0.45),
        polygon(minus_x, RIGHT, *edge),
        polygon(minus_y, LEFT, *edge),
        polygon(top_face, TOP, *edge),
        arrow(center, point(172, 0, -t), AXES["x"], axis_width),
        arrow(center, point(0, 172, -t), AXES["y"], axis_width),
        arrow(z_from, (ORIGIN[0], z_end), AXES["z"], axis_width),
        ring(152, -t / 2, True, RING, ring_width, 0.95),
        "  </g>" if small else "",
        "</svg>",
        "",
    ]
    return "\n".join(part for part in parts if part)


def main():
    """Write the SVG and every PNG size."""
    (HERE / "sensor_config.svg").write_text(icon())
    for size in SIZES:
        cairosvg.svg2png(bytestring=icon(small=size <= 48).encode(),
                         write_to=str(HERE / f"sensor_config-{size}.png"),
                         output_width=size, output_height=size)


if __name__ == "__main__":
    main()
