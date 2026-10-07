import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_map_extra as base  


MAP_NAME = "map_s_trap"
WALL_THICK = 0.15
WALL_HEIGHT = 2.5
ROBOT_WIDTH = 0.31
COSTMAP_INFLATION = 0.40
ROBOT_TO_COSTMAP_CLEARANCE = 0.05
CURVE_RASTER_ALLOWANCE = 0.03
# wall centreline -> path centreline
PASSAGE_HALF_GAP = (0.5 * WALL_THICK + 0.5 * ROBOT_WIDTH +
                    COSTMAP_INFLATION + ROBOT_TO_COSTMAP_CLEARANCE)
# wall  -> path centreline
PASSAGE_HALF_FREE = PASSAGE_HALF_GAP - 0.5 * WALL_THICK + CURVE_RASTER_ALLOWANCE
S_PASSAGE_HALF_GAP = PASSAGE_HALF_GAP + CURVE_RASTER_ALLOWANCE

ROOM_HALF_X = 4.60
ROOM_BOTTOM_Y = -4.60
MAP_W = MAP_H = 384
RESOLUTION = 0.05
ORIGIN = (-10.0, -10.0)

ROOM_FLOOR_SURFACE = ROOM_BOTTOM_Y + 0.5 * WALL_THICK
# Height of the pocket the robot spawns in.
SPAWN_POCKET_H = 1.96
FLOOD_SEED = (0.0, ROOM_FLOOR_SURFACE + 0.5 * SPAWN_POCKET_H)

# Centreline of the S passage.
S_BEZIER = ((-1.55, 0.60), (-0.45, 0.60), (-0.35, -1.00), (0.85, -1.00))
_S_LOWEST = S_BEZIER[3][1] - S_PASSAGE_HALF_GAP - 0.5 * WALL_THICK
S_SHIFT_Y = (ROOM_FLOOR_SURFACE + SPAWN_POCKET_H) - _S_LOWEST
# Top surface of the upper S wall.
S_TOP_SURFACE = (S_BEZIER[0][1] + S_SHIFT_Y + S_PASSAGE_HALF_GAP +
                 0.5 * WALL_THICK)

# Goal trap.
U_BASE_RADIUS = 1.25
U_BASE_ARM_LEN = 1.00
U_SCALE = 1.05  # 0.70
U_RADIUS = U_BASE_RADIUS * U_SCALE
U_ARM_LEN = 0.675  # 0.45 m
U_ARC_SEGMENTS = 30
# Additional free space between the bottom of the U and the upper S wall.
U_EXTRA_LOWER_CLEARANCE = 0.30
# Extra band above the U mouth so DWA can turn and align with the opening.
U_TOP_TURN_CLEARANCE = 0.60
_U_REFERENCE_BOTTOM_Y = (S_TOP_SURFACE + 2.0 * PASSAGE_HALF_FREE +
                         0.5 * WALL_THICK + U_EXTRA_LOWER_CLEARANCE)
U_CENTER_X = -2.00
U_CENTER_Y = (_U_REFERENCE_BOTTOM_Y + U_BASE_RADIUS +
              0.5 * (U_BASE_ARM_LEN - U_BASE_RADIUS))
U_ARC_Y = U_CENTER_Y + 0.5 * (U_RADIUS - U_ARM_LEN)
U_MOUTH_Y = U_ARC_Y + U_ARM_LEN
# Vertical arms.
U_APEX_Y = U_MOUTH_Y
GOAL = (U_CENTER_X, U_CENTER_Y + 0.225)
ROOM_TOP_Y = (U_APEX_Y + 2.0 * PASSAGE_HALF_FREE + 0.5 * WALL_THICK +
              U_TOP_TURN_CLEARANCE)
ROOM_CENTER_Y = 0.5 * (ROOM_TOP_Y + ROOM_BOTTOM_Y)
ROOM_LEN_X = 2.0 * ROOM_HALF_X + WALL_THICK
ROOM_LEN_Y = (ROOM_TOP_Y - ROOM_BOTTOM_Y) + WALL_THICK


def segment(cx1, cy1, cx2, cy2, name):
    """Return one wall box whose long axis connects two world points."""
    dx, dy = cx2 - cx1, cy2 - cy1
    length = math.hypot(dx, dy)
    return ((cx1 + cx2) / 2.0, (cy1 + cy2) / 2.0, length + 0.04,
            WALL_THICK, math.atan2(dy, dx), name)


def arc_segments(cx, cy, radius, start_deg, end_deg, count, name):
    """Approximate an arc with overlapping boxes following its tangent."""
    step = math.radians(end_deg - start_deg) / count
    # Slight overlap keeps the occupancy grid and Gazebo wall watertight.
    length = 2.0 * radius * math.sin(abs(step) / 2.0) * 1.25
    boxes = []
    for i in range(count):
        theta = math.radians(start_deg) + (i + 0.5) * step
        boxes.append((cx + radius * math.cos(theta),
                      cy + radius * math.sin(theta),
                      length, WALL_THICK, theta + math.pi / 2.0,
                      "%s_%02d" % (name, i)))
    return boxes


def bezier_parallel_walls(points, half_gap, count):
    """Create two constant-offset walls around a cubic Bezier centreline.

    Offset is measured along the local normal, so the free curved gap does not
    widen or narrow through the bend.  Both end tangents are horizontal, which
    lets the four straight arms meet the curves without a corner.
    """
    p0, p1, p2, p3 = points

    def point_and_tangent(t):
        u = 1.0 - t
        x = (u**3 * p0[0] + 3*u*u*t * p1[0] +
             3*u*t*t * p2[0] + t**3 * p3[0])
        y = (u**3 * p0[1] + 3*u*u*t * p1[1] +
             3*u*t*t * p2[1] + t**3 * p3[1])
        dx = (3*u*u * (p1[0] - p0[0]) +
              6*u*t * (p2[0] - p1[0]) +
              3*t*t * (p3[0] - p2[0]))
        dy = (3*u*u * (p1[1] - p0[1]) +
              6*u*t * (p2[1] - p1[1]) +
              3*t*t * (p3[1] - p2[1]))
        norm = math.hypot(dx, dy)
        return x, y, -dy / norm, dx / norm

    upper_points, lower_points = [], []
    for i in range(count + 1):
        x, y, nx, ny = point_and_tangent(i / float(count))
        upper_points.append((x + half_gap * nx, y + half_gap * ny))
        lower_points.append((x - half_gap * nx, y - half_gap * ny))

    def boxes(polyline, prefix):
        return [segment(*polyline[i], *polyline[i + 1], "%s_%02d" % (prefix, i))
                for i in range(len(polyline) - 1)]

    return upper_points, lower_points, boxes(upper_points, "S_upper_curve"), boxes(lower_points, "S_lower_curve")


def build_boxes():
    outer = [
        (0.0, ROOM_BOTTOM_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_bottom"),
        (0.0, ROOM_TOP_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_top"),
        (-ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_left"),
        (ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_right"),
    ]

    # Two opposing hooks.
    bezier = tuple((px, py + S_SHIFT_Y) for px, py in S_BEZIER)
    upper_pts, lower_pts, upper_curve, lower_curve = bezier_parallel_walls(
        bezier, half_gap=S_PASSAGE_HALF_GAP, count=28)

    upper_start, upper_end = upper_pts[0], upper_pts[-1]
    lower_start, lower_end = lower_pts[0], lower_pts[-1]
    s_walls = [
        # Left-facing hook: upper/lower arms plus the lower offset curve.
        segment(-ROOM_HALF_X + WALL_THICK / 2.0, lower_start[1],
                lower_start[0], lower_start[1], "Left_hook_upper_arm"),
        *lower_curve,
        segment(-ROOM_HALF_X + WALL_THICK / 2.0, lower_end[1],
                lower_end[0], lower_end[1], "Left_hook_lower_arm"),
        # Right-facing hook: upper/lower arms plus the upper offset curve.
        segment(upper_start[0], upper_start[1],
                ROOM_HALF_X - WALL_THICK / 2.0, upper_start[1], "Right_hook_upper_arm"),
        *upper_curve,
        segment(upper_end[0], upper_end[1],
                ROOM_HALF_X - WALL_THICK / 2.0, upper_end[1], "Right_hook_lower_arm"),
    ]

    # Goal trap.
    arm_cy = U_ARC_Y + U_ARM_LEN / 2.0
    u_walls = [
        *arc_segments(U_CENTER_X, U_ARC_Y, U_RADIUS, 180.0, 360.0, U_ARC_SEGMENTS,
                      "Goal_U_bottom"),
        (U_CENTER_X - U_RADIUS, arm_cy, U_ARM_LEN, WALL_THICK, math.pi / 2.0,
         "Goal_U_left"),
        (U_CENTER_X + U_RADIUS, arm_cy, U_ARM_LEN, WALL_THICK, math.pi / 2.0,
         "Goal_U_right"),
    ]
    return outer, s_walls, u_walls


def wall_link(name, cx, cy, lx, ly, yaw):
    size = "%g %g %g" % (lx, ly, WALL_HEIGHT)
    half_h = WALL_HEIGHT / 2.0
    return """      <link name='{name}'>
        <collision name='{name}_Collision'>
          <geometry><box><size>{size}</size></box></geometry>
          <pose>0 0 {half_h} 0 -0 0</pose>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='{name}_Visual'>
          <pose>0 0 {half_h} 0 -0 0</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
        </visual>
        <pose>{cx:.6f} {cy:.6f} 0 0 -0 {yaw:.6f}</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
""".format(name=name, size=size, half_h=half_h, cx=cx, cy=cy, yaw=yaw)


def write_world(path, boxes_by_model):
    out = [base.WORLD_HEAD]
    models = [("ground_plane", "0 0 0 0 -0 0", [("link", "0 0 0 0 -0 0")])]
    for model_name, boxes in boxes_by_model:
        out.append("    <model name='%s'>\n      <pose>0 0 0 0 -0 0</pose>\n" % model_name)
        links = []
        for cx, cy, lx, ly, yaw, name in boxes:
            out.append(wall_link(name, cx, cy, lx, ly, yaw))
            links.append((name, "%.6f %.6f 0 0 -0 %.6f" % (cx, cy, yaw)))
        out.append("      <static>1</static>\n    </model>\n")
        models.append((model_name, "0 0 0 0 -0 0", links))
    out.append(base.state_block(models))
    out.append(base.WORLD_TAIL)
    with open(path, "w") as stream:
        stream.write("".join(out))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    map_dir = os.path.join(pkg, "maps", MAP_NAME)
    os.makedirs(map_dir, exist_ok=True)

    outer, s_walls, u_walls = build_boxes()
    all_boxes = outer + s_walls + u_walls

    # Configure the reused rasterizer.
    base.MAP_W = base.MAP_H = MAP_W
    base.RESOLUTION = RESOLUTION
    base.ORIGIN = ORIGIN
    base.FLOOD_SEED = FLOOD_SEED
    base.UNIT_BOXES = []

    write_world(os.path.join(pkg, "worlds", MAP_NAME + ".world"),
                ((MAP_NAME + "_outer", outer),
                 (MAP_NAME + "_s_walls", s_walls),
                 (MAP_NAME + "_goal_u", u_walls)))
    image = base.rasterize(all_boxes)
    base.write_pgm(os.path.join(map_dir, MAP_NAME + ".pgm"), image)
    base.write_map_yaml(os.path.join(map_dir, MAP_NAME + ".yaml"), MAP_NAME + ".pgm")

    print("world: %s" % os.path.join(pkg, "worlds", MAP_NAME + ".world"))
    print("map:   %s/%s.{pgm,yaml}" % (map_dir, MAP_NAME))
    print("room:  x [%.2f, %.2f], y [%.2f, %.2f]"
          % (-ROOM_HALF_X, ROOM_HALF_X, ROOM_BOTTOM_Y, ROOM_TOP_Y))
    print("start: (%.2f, %.2f), goal: (%.2f, %.2f)"
          % (FLOOD_SEED[0], FLOOD_SEED[1], GOAL[0], GOAL[1]))
    print("S hooks shifted by %+.2f m; corridor under the bowl: %.2f m free"
          % (S_SHIFT_Y,
             (U_ARC_Y - U_RADIUS - 0.5 * WALL_THICK) - S_TOP_SURFACE))
    print("spawn pocket %.2f m tall, robot centred with %.3f m to each side"
          % (SPAWN_POCKET_H, 0.5 * SPAWN_POCKET_H))
    print("required clearance along every corridor: %.3f m "
          "(inflation %.2f + %.2f margin + half footprint %.3f)"
          % (PASSAGE_HALF_FREE - CURVE_RASTER_ALLOWANCE, COSTMAP_INFLATION,
             ROBOT_TO_COSTMAP_CLEARANCE, 0.5 * ROBOT_WIDTH))


if __name__ == "__main__":
    main()
