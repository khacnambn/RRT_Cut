#!/usr/bin/env python3
"""Generate an hourglass/U-trap benchmark based on the supplied sketch."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_map_extra as base  # noqa: E402
import generate_map_s_trap as geom  # noqa: E402


MAP_NAME = "map_hourglass_u"
MAP_W = MAP_H = 384
RESOLUTION = 0.05
ORIGIN = (-10.0, -10.0)

WALL_THICK = 0.15
ROBOT_WIDTH = 0.31
INFLATION_RADIUS = 0.40
MIN_CLEARANCE = 0.05
RASTER_ALLOWANCE_PER_SIDE = 0.03

# Surface gap required by the robot/inflation model is 1.21 m.  Add 0.03 m
# per side so the 0.05 m raster cannot reduce the requested 5 cm clearance.
REQUIRED_SURFACE_GAP = ROBOT_WIDTH + 2.0 * (INFLATION_RADIUS + MIN_CLEARANCE)
NARROW_CENTRE_GAP = (
    REQUIRED_SURFACE_GAP + WALL_THICK + 2.0 * RASTER_ALLOWANCE_PER_SIDE
)

ROOM_HALF_X = 4.60
ROOM_BOTTOM_Y = -4.60
ROOM_TOP_Y = 4.60
ROOM_CENTER_Y = 0.0
ROOM_LEN_X = 2.0 * ROOM_HALF_X + WALL_THICK
ROOM_LEN_Y = ROOM_TOP_Y - ROOM_BOTTOM_Y + WALL_THICK

DIVIDER_Y = 3.20
DIVIDER_INNER_X = 1.20
HOURGLASS_NECK_X = 0.5 * NARROW_CENTRE_GAP
HOURGLASS_LEFT_ARM_X = 1.70
HOURGLASS_RIGHT_ARM_X = 1.85
HOURGLASS_ARM_TOP_Y = 0.60
HOURGLASS_LEFT_ARM_BOTTOM_Y = -1.80
HOURGLASS_RIGHT_ARM_BOTTOM_Y = 0.00
SIDE_WALL_LEFT_X = HOURGLASS_LEFT_ARM_X + NARROW_CENTRE_GAP
SIDE_WALL_RIGHT_X = HOURGLASS_RIGHT_ARM_X + NARROW_CENTRE_GAP
SIDE_WALL_TOP_Y = 1.80

U_CENTER_X = 0.30
U_ARC_Y = -1.65
U_RADIUS = 1.00
U_ARM_LEN = 0.90
U_ARC_SEGMENTS = 30

START = (-3.85, -3.45)
START_YAW = math.pi / 2.0
GOAL = (0.30, -1.85)
FLOOD_SEED = START


def cubic_points(p0, p1, p2, p3, count):
    points = []
    for i in range(count + 1):
        t = i / float(count)
        u = 1.0 - t
        points.append((
            u**3 * p0[0] + 3.0 * u*u*t * p1[0]
            + 3.0 * u*t*t * p2[0] + t**3 * p3[0],
            u**3 * p0[1] + 3.0 * u*u*t * p1[1]
            + 3.0 * u*t*t * p2[1] + t**3 * p3[1],
        ))
    return points


def polyline_segments(points, prefix):
    return [
        geom.segment(*points[i], *points[i + 1], "%s_%02d" % (prefix, i))
        for i in range(len(points) - 1)
    ]


def mirrored(points):
    return [(-x, y) for x, y in points]


def build_boxes():
    outer = [
        (0.0, ROOM_BOTTOM_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_bottom"),
        (0.0, ROOM_TOP_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_top"),
        (-ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_left"),
        (ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_right"),
    ]

    divider = [
        geom.segment(-ROOM_HALF_X + WALL_THICK / 2.0, DIVIDER_Y,
                     -DIVIDER_INNER_X, DIVIDER_Y, "Divider_left"),
        geom.segment(DIVIDER_INNER_X, DIVIDER_Y,
                     ROOM_HALF_X - WALL_THICK / 2.0, DIVIDER_Y,
                     "Divider_right"),
    ]

    left_upper = cubic_points(
        (-DIVIDER_INNER_X, DIVIDER_Y), (-DIVIDER_INNER_X, 2.72),
        (-HOURGLASS_NECK_X, 2.76),
        (-HOURGLASS_NECK_X, 2.25), 16
    )
    left_lower = cubic_points(
        (-HOURGLASS_NECK_X, 2.25),
        (-HOURGLASS_NECK_X, 1.68),
        (-HOURGLASS_LEFT_ARM_X, 1.50),
        (-HOURGLASS_LEFT_ARM_X, HOURGLASS_ARM_TOP_Y), 22
    )
    right_upper = mirrored(left_upper)
    right_lower = cubic_points(
        (HOURGLASS_NECK_X, 2.25),
        (HOURGLASS_NECK_X, 1.68),
        (HOURGLASS_RIGHT_ARM_X, 1.50),
        (HOURGLASS_RIGHT_ARM_X, HOURGLASS_ARM_TOP_Y), 22
    )

    hourglass = [
        *polyline_segments(left_upper, "Hourglass_left_upper"),
        *polyline_segments(left_lower, "Hourglass_left_lower"),
        geom.segment(-HOURGLASS_LEFT_ARM_X, HOURGLASS_ARM_TOP_Y,
                     -HOURGLASS_LEFT_ARM_X, HOURGLASS_LEFT_ARM_BOTTOM_Y,
                     "Hourglass_left_arm"),
        *polyline_segments(right_upper, "Hourglass_right_upper"),
        *polyline_segments(right_lower, "Hourglass_right_lower"),
        geom.segment(HOURGLASS_RIGHT_ARM_X, HOURGLASS_ARM_TOP_Y,
                     HOURGLASS_RIGHT_ARM_X, HOURGLASS_RIGHT_ARM_BOTTOM_Y,
                     "Hourglass_right_arm"),
    ]

    side_walls = [
        geom.segment(-SIDE_WALL_LEFT_X, ROOM_BOTTOM_Y + WALL_THICK / 2.0,
                     -SIDE_WALL_LEFT_X, SIDE_WALL_TOP_Y, "Side_wall_left"),
        geom.segment(SIDE_WALL_RIGHT_X, ROOM_BOTTOM_Y + WALL_THICK / 2.0,
                     SIDE_WALL_RIGHT_X, SIDE_WALL_TOP_Y, "Side_wall_right"),
    ]

    # Downward-open U: upper semicircular cap and two descending legs.
    arm_cy = U_ARC_Y - U_ARM_LEN / 2.0
    u_trap = [
        *geom.arc_segments(U_CENTER_X, U_ARC_Y, U_RADIUS,
                           0.0, 180.0, U_ARC_SEGMENTS, "Goal_U_top"),
        (U_CENTER_X - U_RADIUS, arm_cy, U_ARM_LEN, WALL_THICK,
         math.pi / 2.0, "Goal_U_left"),
        (U_CENTER_X + U_RADIUS, arm_cy, U_ARM_LEN, WALL_THICK,
         math.pi / 2.0, "Goal_U_right"),
    ]
    return outer, divider, hourglass, side_walls, u_trap


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    map_dir = os.path.join(pkg, "maps", MAP_NAME)
    os.makedirs(map_dir, exist_ok=True)

    groups = build_boxes()
    all_boxes = [box for group in groups for box in group]
    base.MAP_W = base.MAP_H = MAP_W
    base.RESOLUTION = RESOLUTION
    base.ORIGIN = ORIGIN
    base.FLOOD_SEED = FLOOD_SEED
    base.UNIT_BOXES = []

    geom.write_world(
        os.path.join(pkg, "worlds", MAP_NAME + ".world"),
        ((MAP_NAME + "_outer", groups[0]),
         (MAP_NAME + "_divider", groups[1]),
         (MAP_NAME + "_hourglass", groups[2]),
         (MAP_NAME + "_side_walls", groups[3]),
         (MAP_NAME + "_goal_u", groups[4])),
    )
    image = base.rasterize(all_boxes)
    base.write_pgm(os.path.join(map_dir, MAP_NAME + ".pgm"), image)
    base.write_map_yaml(
        os.path.join(map_dir, MAP_NAME + ".yaml"), MAP_NAME + ".pgm"
    )

    left_surface_gap = SIDE_WALL_LEFT_X - HOURGLASS_LEFT_ARM_X - WALL_THICK
    right_surface_gap = SIDE_WALL_RIGHT_X - HOURGLASS_RIGHT_ARM_X - WALL_THICK
    surface_gap = min(left_surface_gap, right_surface_gap)
    post_inflation_width = surface_gap - 2.0 * INFLATION_RADIUS
    actual_clearance = 0.5 * (post_inflation_width - ROBOT_WIDTH)
    print("world: %s" % os.path.join(pkg, "worlds", MAP_NAME + ".world"))
    print("map:   %s/%s.{pgm,yaml}" % (map_dir, MAP_NAME))
    print("start: (%.3f, %.3f), yaw %.6f" % (START[0], START[1], START_YAW))
    print("goal:  (%.3f, %.3f)" % GOAL)
    print("narrow wall-surface gap: %.3f m" % surface_gap)
    print("free width after inflation: %.3f m" % post_inflation_width)
    print("robot clearance at narrowest point: %.3f m per side"
          % actual_clearance)


if __name__ == "__main__":
    main()
