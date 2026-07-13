import math
import time
import os
from collections import deque

# Global Settings

gravitational_acceleration = 9.81
fluid_density = 1.225

HISTORY_LENGTH = 5400
SIM_FPS = 60

class Vector2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2d(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2d(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Vector2d(self.x * other, self.y * other)

        elif isinstance(other, (list, tuple)) and len(other) == 2:
            return Vector2d(self.x * other[0], self.y * other[1])

        elif isinstance(other, Vector2d):
            return Vector2d(self.x * other.x, self.y * other.y)

        else:
            raise TypeError("Vector Multiply Error")

    __rmul__ = __mul__

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Vector2d(self.x / other, self.y / other)

        elif isinstance(other, (list, tuple)) and len(other) == 2:
            return Vector2d(self.x / other[0], self.y / other[1])

        elif isinstance(other, Vector2d):
            return Vector2d(self.x / other.x,  self.y / other.y)

        else:
            raise TypeError("Vector Division Error")

    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2)

    def angle(self):
        return math.degrees(math.atan2(self.y, self.x))

    def normalise(self):
        magnitude = self.magnitude()
        if magnitude == 0:
            return Vector2d(0, 0)
        return self / magnitude

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def __neg__(self):
        return Vector2d(-self.x, -self.y)

    def cross(self, other):
        if isinstance(other, Vector2d):
            return self.x * other.y - self.y * other.x
        elif isinstance(other, (int, float)):
            return Vector2d(other * self.y, -other * self.x)
        else:
            raise TypeError("Vector Cross Error")

    def rcross(self, scalar):
        return Vector2d(-scalar * self.y, scalar * self.x)

    def __str__(self):
        return f"({self.x}, {self.y})"


def force_resolve(magnitude, angle, in_radians=False):
    if in_radians == False:
        angle = math.radians(angle)
    return Vector2d(magnitude * math.sin(angle), magnitude * math.cos(angle))


def calculate_axes_from_corners(corners):
    axes = []

    for i in range(4):
        point1 = corners[i]
        point2 = corners[(i + 1) % 4]

        edge = point2 - point1

        axis = Vector2d(-edge.y, edge.x).normalise()
        axes.append(axis)
    return axes[:2]


def calculate_axes_from_block(block):
    corners = block.calculate_corners()
    axes = calculate_axes_from_corners(corners)
    return axes


def project_corners(corners, axis):
    projections = []
    for corner in corners:
        projections.append(corner.dot(axis))

    return min(projections), max(projections)


def overlap_check(min1, max1, min2, max2):
    return min(max1, max2) - max(min1, min2)


def check_collision(block1, block2):

    axes1 = calculate_axes_from_block(block1)
    axes2 = calculate_axes_from_block(block2)
    axes = axes1 + axes2

    min_overlap = float("inf")
    best_axis = None

    corners1 = block1.calculate_corners()
    corners2 = block2.calculate_corners()

    for axis in axes:

        min1, max1 = project_corners(corners1, axis)
        min2, max2 = project_corners(corners2, axis)

        overlap = overlap_check(min1, max1, min2, max2)

        if overlap <= 0:
            return False, None, None

        if overlap < min_overlap:
            min_overlap = overlap
            best_axis = axis

    center_delta = block2.position - block1.position

    if center_delta.dot(best_axis) < 0:
        best_axis = -best_axis

    return True, best_axis, min_overlap


def point_in_block(point, block):
    d = point - block.position
    local_axes = calculate_axes_from_block(block)
    axis_h = local_axes[0]
    axis_w = Vector2d(-axis_h.y, axis_h.x)
    proj_w = abs(d.dot(axis_w))
    proj_h = abs(d.dot(axis_h))
    return proj_w <= (block.dimensions[0] / 2.0) and proj_h <= (block.dimensions[1] / 2.0)


class Material:
    def __init__(self, static_friction=0.6, kinetic_friction=0.3, restitution=0.1):
        self.static_friction = static_friction
        self.kinetic_friction = kinetic_friction
        self.restitution = restitution


def combine_materials(m1, m2):
    static_friction = math.sqrt(max(m1.static_friction, 0.0) * max(m2.static_friction, 0.0))
    kinetic_friction = math.sqrt(max(m1.kinetic_friction, 0.0) * max(m2.kinetic_friction, 0.0))
    restitution = max(m1.restitution, m2.restitution)
    return static_friction, kinetic_friction, restitution


class Gravity:
    name = "Gravity"

    def __init__(self, g=9.81):
        self.g = g

    def calculate(self, block):
        return Vector2d(0, block.mass * -self.g)


class Drag:
    name = "Drag"

    def __init__(self, fluid_density=1.225):
        self.rho = fluid_density

    def calculate(self, block, drag_coefficient=0.47):
        velocity = block.velocity.magnitude()
        if velocity == 0:
            return Vector2d(0, 0)

        vel_dir = block.velocity.normalise()
        axes = calculate_axes_from_block(block)

        half_w = block.dimensions[0] / 2.0
        half_h = block.dimensions[1] / 2.0
        frontal_length = (half_w * abs(axes[1].dot(vel_dir)) +
                           half_h * abs(axes[0].dot(vel_dir))) * 2

        area = frontal_length
        drag_magnitude = 0.5 * self.rho * drag_coefficient * area * (velocity ** 2)

        drag_direction = -vel_dir
        return drag_direction * drag_magnitude


class ConstantForce:
    """A permanent externally-applied force, e.g. someone leaning on a block."""

    def __init__(self, vector, name="Push"):
        self.vector = vector
        self.name = name

    def calculate(self, block):
        return self.vector


class DragSpring:
    """
    Soft-pull mouse interaction. Instead of teleporting the grabbed block
    (which can punch through obstacles), this applies a spring+damper
    force at the exact point on the block that was grabbed, pulling that
    point toward the current mouse position. Because it's just another
    force in the normal physics loop, dragged blocks keep colliding
    properly - they push against obstacles instead of tunnelling through.

    local_offset is stored in the block's own unrotated frame so the
    grabbed point stays physically fixed to the block as it rotates.
    """
    name = "Pull"

    def __init__(self, target_block, local_offset, get_mouse_world_pos,
                 stiffness_per_mass=40.0, damping_per_mass=8.0,
                 max_force_per_mass=200.0):
        self.target_block = target_block
        self.local_offset = local_offset
        self.get_mouse_world_pos = get_mouse_world_pos
        self.stiffness_per_mass = stiffness_per_mass
        self.damping_per_mass = damping_per_mass
        self.max_force_per_mass = max_force_per_mass

    def world_grab_point(self, block):
        angle_rad = math.radians(block.angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        ox, oy = self.local_offset.x, self.local_offset.y
        r = Vector2d(ox * cos_a - oy * sin_a, ox * sin_a + oy * cos_a)
        return block.position + r, r

    def calculate(self, block):
        if block is not self.target_block or block.mass == 0.0:
            return Vector2d(0, 0)

        grab_point, r = self.world_grab_point(block)
        mouse_pos = self.get_mouse_world_pos()

        displacement = mouse_pos - grab_point
        point_velocity = block.velocity + r.rcross(block.angular_velocity)

        stiffness = self.stiffness_per_mass * block.mass
        damping = self.damping_per_mass * block.mass

        force = (displacement * stiffness) + (point_velocity * -damping)

        max_force = self.max_force_per_mass * block.mass
        mag = force.magnitude()
        if mag > max_force and mag > 0:
            force = force * (max_force / mag)

        return force


FORCE_COLORS = {
    "Gravity": "#16a085",
    "Drag": "#2980b9",
    "Friction": "#c0392b",
    "Pull": "#e67e22",
}
CUSTOM_FORCE_COLOR = "#8e44ad"


def get_force_color(name):
    return FORCE_COLORS.get(name, CUSTOM_FORCE_COLOR)


def find_contact_points(block1, block2):
    corners1 = block1.calculate_corners()
    corners2 = block2.calculate_corners()

    contact_points = []

    for corner in corners1:
        if point_in_block(corner, block2):
            contact_points.append(corner)

    for corner in corners2:
        if point_in_block(corner, block1):
            contact_points.append(corner)

    if not contact_points:
        return [(block1.position + block2.position) * 0.5]

    return contact_points


def apply_collision_impulse(obj1, obj2, normal, contact_point, target_velocity=0.0):

    r1 = contact_point - obj1.position
    r2 = contact_point - obj2.position

    v1 = obj1.velocity + r1.rcross(obj1.angular_velocity)
    v2 = obj2.velocity + r2.rcross(obj2.angular_velocity)

    relative_velocity = v2 - v1
    vel_along_normal = relative_velocity.dot(normal)

    if vel_along_normal >= target_velocity:
        return 0.0

    r1_cross_n = r1.cross(normal)
    r2_cross_n = r2.cross(normal)

    inv_mass1 = 0.0 if obj1.mass == 0.0 else 1.0 / obj1.mass
    inv_mass2 = 0.0 if obj2.mass == 0.0 else 1.0 / obj2.mass
    inv_inertia1 = 0.0 if obj1.mass == 0.0 else 1.0 / obj1.inertia
    inv_inertia2 = 0.0 if obj2.mass == 0.0 else 1.0 / obj2.inertia

    denom = inv_mass1 + inv_mass2 + \
            (r1_cross_n ** 2 * inv_inertia1) + (r2_cross_n ** 2 * inv_inertia2)

    if denom == 0:
        return 0.0

    j = (target_velocity - vel_along_normal) / denom

    impulse = normal * j

    obj1.velocity = obj1.velocity - (impulse * inv_mass1)
    obj2.velocity = obj2.velocity + (impulse * inv_mass2)

    obj1.angular_velocity = obj1.angular_velocity - r1.cross(impulse) * inv_inertia1
    obj2.angular_velocity = obj2.angular_velocity + r2.cross(impulse) * inv_inertia2

    return j


class Block:
    def __init__(self, mass, dimensions, position, velocity, forces, angle, angular_velocity=0.0,
                 material=None, name=None, color=None):
        self.mass = mass
        self.dimensions = dimensions
        self.position = Vector2d(position.x, position.y)
        self.velocity = Vector2d(velocity.x, velocity.y)
        self.forces = Vector2d(forces.x, forces.y)
        self.angle = angle
        self.angular_velocity = angular_velocity
        self.torque = 0.0

        self.inertia = (1.0 / 12.0) * self.mass * (self.dimensions[0] ** 2 + self.dimensions[1] ** 2)
        self.past_velocity = []

        self.material = material if material is not None else Material()
        self.custom_forces = []
        self.force_breakdown = {}
        self.friction_accum = Vector2d(0, 0)
        self.name = name if name is not None else f"block_{id(self) % 10000}"
        self.color = color
        self.selected = False
        self.stored_mass = None

        self._sim_time = 0.0
        self.history = {
            "t": deque(maxlen=HISTORY_LENGTH),
            "pos_x": deque(maxlen=HISTORY_LENGTH),
            "pos_y": deque(maxlen=HISTORY_LENGTH),
            "vel": deque(maxlen=HISTORY_LENGTH),
            "angle": deque(maxlen=HISTORY_LENGTH),
            "ang_vel": deque(maxlen=HISTORY_LENGTH),
            "momentum": deque(maxlen=HISTORY_LENGTH),
        }

    @property
    def volume(self):
        return self.dimensions[0] * self.dimensions[1]

    @property
    def density(self):
        return self.mass / self.volume

    @property
    def acceleration(self):
        if self.mass == 0.0:
            return Vector2d(0, 0)
        return self.forces / self.mass

    @property
    def angular_acceleration(self):
        if self.inertia == 0:
            return 0.0
        return self.torque / self.inertia

    @property
    def momentum(self):
        return self.velocity * self.mass

    def recompute_inertia(self):
        self.inertia = (1.0 / 12.0) * self.mass * (self.dimensions[0] ** 2 + self.dimensions[1] ** 2)

    def set_mass(self, new_mass):
        self.mass = max(0.0, new_mass)
        self.recompute_inertia()

    def set_dimensions(self, width, height):
        self.dimensions = (max(0.01, width), max(0.01, height))
        self.recompute_inertia()

    def calculate_corners(self):
        half_width = self.dimensions[0] / 2
        half_height = self.dimensions[1] / 2

        relative_corners = [
            Vector2d(-half_width, -half_height),
            Vector2d(half_width, -half_height),
            Vector2d(half_width, half_height),
            Vector2d(-half_width, half_height)
        ]

        block_angle_radians = math.radians(self.angle)
        cos_angle = math.cos(block_angle_radians)
        sin_angle = math.sin(block_angle_radians)

        world_corners = []
        for corner in relative_corners:
            rotated_corner = Vector2d(
                corner.x * cos_angle - corner.y * sin_angle,
                corner.x * sin_angle + corner.y * cos_angle
            )
            world_corners.append(self.position + rotated_corner)

        return world_corners

    def log_history(self, dt):
        self._sim_time += dt
        h = self.history
        h["t"].append(self._sim_time)
        h["pos_x"].append(self.position.x)
        h["pos_y"].append(self.position.y)
        h["vel"].append(self.velocity.magnitude())
        h["angle"].append(self.angle)
        h["ang_vel"].append(self.angular_velocity)
        h["momentum"].append(self.momentum.magnitude())

    def update_position(self, frame_rate, external_forces):
        dt = 1 / frame_rate

        if self.mass == 0.0:
            self.velocity = Vector2d(0, 0)
            self.angular_velocity = 0.0
            self.force_breakdown = {}
            self.log_history(dt)
            return

        self.forces = Vector2d(0, 0)
        self.torque = 0.0
        self.force_breakdown = {}

        all_forces = list(external_forces) + list(self.custom_forces)

        for force in all_forces:
            f = force.calculate(self)
            self.forces = self.forces + f
            key = getattr(force, "name", force.__class__.__name__)
            if key in self.force_breakdown:
                self.force_breakdown[key] = self.force_breakdown[key] + f
            else:
                self.force_breakdown[key] = f

        self.velocity = self.velocity + self.acceleration * dt
        self.position = self.position + self.velocity * dt

        self.angular_velocity = self.angular_velocity + self.angular_acceleration * dt
        self.angle = self.angle + math.degrees(self.angular_velocity * dt)

        self.log_history(dt)

    def __str__(self):
        return f'A Block of mass {self.mass}, dimensions {self.dimensions[0]} by {self.dimensions[1]} located x:{self.position.x} y:{self.position.y} moving x:{self.velocity.x}/s y:{self.velocity.y}/s'

    def advanced_print(self):
        print("mass:", self.mass)
        print("dimensions:", self.dimensions)
        print("position:", self.position)
        print("velocity:", self.velocity)
        print("forces:", self.forces)
        print("past_velocity:", self.past_velocity)
        print("volume:", self.volume)
        print("density:", self.density)
        print("acceleration:", self.acceleration)
        print("momentum:", self.momentum)


def resolve_penetration(obj1, obj2, normal, penetration):

    inv_mass1 = 0.0 if obj1.mass == 0.0 else 1.0 / obj1.mass
    inv_mass2 = 0.0 if obj2.mass == 0.0 else 1.0 / obj2.mass

    total_inv_mass = inv_mass1 + inv_mass2
    if total_inv_mass == 0:
        return

    slop = 0.01

    percent = 0.2

    max_correction = 0.2

    corrected_penetration = min(max(penetration - slop, 0.0), max_correction)
    magnitude = corrected_penetration / total_inv_mass * percent
    correction = normal * magnitude

    obj1.position = obj1.position - (correction * inv_mass1)
    obj2.position = obj2.position + (correction * inv_mass2)


def apply_friction_impulse(obj1, obj2, normal, contact_point, j_normal):
    if j_normal is None or j_normal == 0.0:
        return

    r1 = contact_point - obj1.position
    r2 = contact_point - obj2.position

    v1 = obj1.velocity + r1.rcross(obj1.angular_velocity)
    v2 = obj2.velocity + r2.rcross(obj2.angular_velocity)
    relative_velocity = v2 - v1

    vel_along_normal = relative_velocity.dot(normal)
    tangent_velocity = relative_velocity - normal * vel_along_normal

    tangent_speed = tangent_velocity.magnitude()
    if tangent_speed < 1e-9:
        return

    tangent = tangent_velocity.normalise()

    mu_static, mu_kinetic, _ = combine_materials(obj1.material, obj2.material)

    r1_cross_t = r1.cross(tangent)
    r2_cross_t = r2.cross(tangent)

    inv_mass1 = 0.0 if obj1.mass == 0.0 else 1.0 / obj1.mass
    inv_mass2 = 0.0 if obj2.mass == 0.0 else 1.0 / obj2.mass
    inv_inertia1 = 0.0 if obj1.mass == 0.0 else 1.0 / obj1.inertia
    inv_inertia2 = 0.0 if obj2.mass == 0.0 else 1.0 / obj2.inertia

    denom = inv_mass1 + inv_mass2 + \
            (r1_cross_t ** 2 * inv_inertia1) + (r2_cross_t ** 2 * inv_inertia2)

    if denom == 0:
        return

    j = -relative_velocity.dot(tangent)
    j /= denom

    static_limit = mu_static * abs(j_normal)

    if abs(j) <= static_limit:
        friction_impulse = tangent * j
    else:
        kinetic_limit = mu_kinetic * abs(j_normal)
        friction_impulse = tangent * max(-kinetic_limit, min(kinetic_limit, j))

    obj1.velocity = obj1.velocity - (friction_impulse * inv_mass1)
    obj2.velocity = obj2.velocity + (friction_impulse * inv_mass2)

    obj1.angular_velocity = obj1.angular_velocity - r1.cross(friction_impulse) * inv_inertia1
    obj2.angular_velocity = obj2.angular_velocity + r2.cross(friction_impulse) * inv_inertia2

    obj1.friction_accum = obj1.friction_accum - friction_impulse
    obj2.friction_accum = obj2.friction_accum + friction_impulse


def world_step(objects, frame_rate, external_forces, velocity_iterations=16, substeps=8, debug_out=None):

    substep_rate = frame_rate * substeps

    RESTITUTION_VELOCITY_THRESHOLD = 0.2

    LINEAR_SLEEP = 0.02
    ANGULAR_SLEEP = 0.02

    for obj in objects:
        obj.friction_accum = Vector2d(0, 0)

    last_frame_debug = {"pairs": []}

    for _ in range(substeps):
        for obj in objects:
            obj.update_position(substep_rate, external_forces)

        pair_contacts = []
        point_contacts = []
        frame_debug = {"pairs": []}

        for i in range(len(objects)):
            for j in range(i + 1, len(objects)):
                obj1 = objects[i]
                obj2 = objects[j]

                hit, normal, penetration = check_collision(obj1, obj2)
                if not hit:
                    continue

                pair_contacts.append((obj1, obj2, normal, penetration))

                contact_pts = find_contact_points(obj1, obj2)
                _, _, restitution = combine_materials(obj1.material, obj2.material)

                for contact_point in contact_pts:

                    r1 = contact_point - obj1.position
                    r2 = contact_point - obj2.position
                    v1 = obj1.velocity + r1.rcross(obj1.angular_velocity)
                    v2 = obj2.velocity + r2.rcross(obj2.angular_velocity)
                    approach_velocity = (v2 - v1).dot(normal)

                    if approach_velocity < -RESTITUTION_VELOCITY_THRESHOLD:
                        target_velocity = -restitution * approach_velocity
                    else:
                        target_velocity = 0.0

                    point_contacts.append((obj1, obj2, normal, contact_point, target_velocity))

                if debug_out is not None:
                    axes = calculate_axes_from_block(obj1) + calculate_axes_from_block(obj2)
                    frame_debug["pairs"].append({
                        "obj1": obj1,
                        "obj2": obj2,
                        "normal": normal,
                        "penetration": penetration,
                        "axes": axes,
                        "contact_points": contact_pts,
                    })

        last_frame_debug = frame_debug

        for _ in range(velocity_iterations):
            for obj1, obj2, normal, contact_point, target_velocity in point_contacts:
                j_normal = apply_collision_impulse(obj1, obj2, normal, contact_point, target_velocity)
                apply_friction_impulse(obj1, obj2, normal, contact_point, j_normal)

        for obj1, obj2, normal, penetration in pair_contacts:
            resolve_penetration(obj1, obj2, normal, penetration)

    for obj in objects:
        if obj.mass == 0.0:
            continue
        if obj.velocity.magnitude() < LINEAR_SLEEP:
            obj.velocity = Vector2d(0.0, 0.0)
        if abs(obj.angular_velocity) < ANGULAR_SLEEP:
            obj.angular_velocity = 0.0


    dt_frame = 1.0 / frame_rate
    for obj in objects:
        if obj.mass == 0.0:
            continue
        obj.force_breakdown["Friction"] = obj.friction_accum / dt_frame

    if debug_out is not None:
        debug_out.clear()
        debug_out.update(last_frame_debug)


# /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
# TKINTER GRAPHICS LAYER
# /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

import tkinter as tk
from tkinter import ttk, filedialog, colorchooser
from screeninfo import get_monitors

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

monitor = get_monitors()[0]
WINDOW_WIDTH = int(monitor.width * 0.8)
WINDOW_HEIGHT = int(monitor.height * 0.8)

TOP_BAR_HEIGHT = 56
LEFT_PANEL_WIDTH = int(WINDOW_WIDTH * 0.16)
RIGHT_PANEL_WIDTH = int(WINDOW_WIDTH * 0.22)

canvas_width = WINDOW_WIDTH - LEFT_PANEL_WIDTH - RIGHT_PANEL_WIDTH
canvas_height = WINDOW_HEIGHT - TOP_BAR_HEIGHT

root = tk.Tk()
root.title("2D Rigid Body Physics Playground")
root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")

# ---------------------------------------------------------------------------
# Top bar: pause / record-to-file / playback-from-file
# ---------------------------------------------------------------------------

top_bar = tk.Frame(root, bg="#d9d9d9")
top_bar.place(x=0, y=0, width=WINDOW_WIDTH, height=TOP_BAR_HEIGHT)

is_paused = False
is_recording = False
is_playing_back = False
playback_index = 0
recording_buffer = []
recording_file = None


def toggle_pause():
    global is_paused
    is_paused = not is_paused
    pause_btn.config(text="Resume" if is_paused else "Pause")


pause_btn = tk.Button(top_bar, text="Pause", command=toggle_pause)
pause_btn.place(x=10, y=13, width=80, height=30)


def write_recording_frame(f, objects):
    parts = []
    for obj in objects:
        parts.append(
            f"{obj.name}:{obj.position.x:.5f},{obj.position.y:.5f},{obj.angle:.5f},"
            f"{obj.velocity.x:.5f},{obj.velocity.y:.5f},{obj.angular_velocity:.5f}"
        )
    f.write(";".join(parts) + "\n")


def load_recording(path):
    frames = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            frame = {}
            for chunk in line.split(";"):
                if ":" not in chunk:
                    continue
                name, values = chunk.split(":", 1)
                px, py, ang, vx, vy, av = (float(v) for v in values.split(","))
                frame[name] = (Vector2d(px, py), ang, Vector2d(vx, vy), av)
            frames.append(frame)
    return frames


def toggle_record():
    global is_recording, recording_file
    if is_playing_back:
        return
    is_recording = not is_recording
    if is_recording:
        os.makedirs("recordings", exist_ok=True)
        filename = os.path.join("recordings", f"recording_{time.strftime('%Y%m%d_%H%M%S')}.txt")
        recording_file = open(filename, "w")
        record_btn.config(text="Stop rec")
        playback_status_label.config(text=f"Recording -> {filename}")
    else:
        if recording_file is not None:
            recording_file.close()
            recording_file = None
        record_btn.config(text="Record")
        playback_status_label.config(text="Recording saved")


record_btn = tk.Button(top_bar, text="Record", command=toggle_record)
record_btn.place(x=100, y=13, width=80, height=30)


def toggle_playback():
    global is_playing_back, playback_index, recording_buffer
    if is_recording:
        return
    if is_playing_back:
        is_playing_back = False
        playback_btn.config(text="Play recording")
        return

    path = filedialog.askopenfilename(
        title="Select a recording to play back",
        initialdir="recordings",
        filetypes=[("Recording files", "*.txt"), ("All files", "*.*")],
    )
    if not path:
        return

    try:
        frames = load_recording(path)
    except (OSError, ValueError):
        playback_status_label.config(text="Could not read that file")
        return

    if not frames:
        playback_status_label.config(text="No frames found in file")
        return

    recording_buffer = frames
    playback_index = 0
    scrub_slider.config(to=max(0, len(recording_buffer) - 1))
    scrub_var.set(0)
    is_playing_back = True
    playback_btn.config(text="Stop playback")
    playback_status_label.config(text=f"Playing {os.path.basename(path)} ({len(frames)} frames)")


playback_btn = tk.Button(top_bar, text="Play recording", command=toggle_playback)
playback_btn.place(x=190, y=13, width=110, height=30)

playback_status_label = tk.Label(top_bar, text="", bg="#d9d9d9")
playback_status_label.place(x=310, y=20)


def on_scrub(val):
    global playback_index
    if is_playing_back:
        playback_index = int(float(val))


scrub_var = tk.IntVar(value=0)
scrub_slider = tk.Scale(top_bar, from_=0, to=0, orient="horizontal",
                         variable=scrub_var, command=on_scrub, showvalue=False, bg="#d9d9d9")
scrub_slider.place(x=WINDOW_WIDTH - 300, y=6, width=280, height=44)

# ---------------------------------------------------------------------------
# Left panel: block list / add block
# ---------------------------------------------------------------------------

left_panel = tk.Frame(root, bg="#cfcfcf")
left_panel.place(x=0, y=TOP_BAR_HEIGHT, width=LEFT_PANEL_WIDTH, height=WINDOW_HEIGHT - TOP_BAR_HEIGHT)

tk.Label(left_panel, text="Blocks", bg="#cfcfcf", font=("Arial", 12, "bold")).pack(anchor="w", padx=8, pady=(10, 4))

block_list_frame = tk.Frame(left_panel, bg="#cfcfcf")
block_list_frame.pack(fill="both", expand=True, padx=8)

block_row_widgets = {}


def refresh_block_list():
    for widget in block_list_frame.winfo_children():
        widget.destroy()
    block_row_widgets.clear()
    for b in sim_objects:
        bg = "#cde3f7" if b.selected else "white"
        row = tk.Frame(block_list_frame, bg=bg)
        row.pack(fill="x", pady=2)
        lbl = tk.Label(row, text=b.name, bg=bg, anchor="w")
        lbl.pack(side="left", fill="x", expand=True, padx=4)
        lbl.bind("<Button-1>", lambda e, b=b: select_block(b))
        del_btn = tk.Button(row, text="x", width=2, command=lambda b=b: remove_block(b))
        del_btn.pack(side="right")
        block_row_widgets[b] = row


def refresh_block_list_highlight():
    for b, row in block_row_widgets.items():
        color = "#cde3f7" if b.selected else "white"
        row.config(bg=color)
        for w in row.winfo_children():
            w.config(bg=color)


def remove_block(b):
    global selected_block
    if b is ground_block:
        return
    if b in sim_objects:
        sim_objects.remove(b)
    if b in canvas_ids:
        main_canvas.delete(canvas_ids[b])
        del canvas_ids[b]
    if selected_block is b:
        select_block(None)
    refresh_block_list()


def open_add_block_dialog():
    dlg = tk.Toplevel(root)
    dlg.title("Add block")
    dlg.geometry("280x420")

    fields = {}

    def add_field(label, default):
        tk.Label(dlg, text=label).pack(anchor="w", padx=10, pady=(8, 0))
        var = tk.StringVar(value=str(default))
        entry = tk.Entry(dlg, textvariable=var)
        entry.pack(fill="x", padx=10)
        fields[label] = var
        return var

    add_field("Mass (kg)", 10.0)
    add_field("Width (m)", 1.0)
    add_field("Height (m)", 1.0)
    add_field("Position X (m)", 0.0)
    add_field("Position Y (m)", 4.0)
    add_field("Angle (deg)", 0.0)

    default_color = tower_colors[len(sim_objects) % len(tower_colors)]
    color_state = {"value": default_color}

    color_row = tk.Frame(dlg)
    color_row.pack(fill="x", padx=10, pady=(10, 0))
    tk.Label(color_row, text="Colour").pack(side="left")
    color_swatch = tk.Canvas(color_row, width=22, height=22, highlightthickness=1, highlightbackground="black")
    color_swatch.pack(side="left", padx=8)
    color_swatch.create_rectangle(0, 0, 22, 22, fill=color_state["value"], outline="", tags="swatch")

    preview = {"id": None}

    def redraw_preview(*_):
        try:
            w = float(fields["Width (m)"].get())
            h = float(fields["Height (m)"].get())
            px = float(fields["Position X (m)"].get())
            py = float(fields["Position Y (m)"].get())
            ang = float(fields["Angle (deg)"].get())
        except ValueError:
            return
        if w <= 0 or h <= 0:
            return
        temp = Block(mass=1.0, dimensions=(w, h), position=Vector2d(px, py),
                     velocity=Vector2d(0, 0), forces=Vector2d(0, 0), angle=ang)
        corners = temp.calculate_corners()
        pts = []
        for c in corners:
            sx, sy = to_screen(c)
            pts.extend([sx, sy])
        if preview["id"] is None:
            preview["id"] = main_canvas.create_polygon(pts, outline=color_state["value"], fill="",
                                                         dash=(5, 3), width=2)
        else:
            main_canvas.coords(preview["id"], *pts)
            main_canvas.itemconfig(preview["id"], outline=color_state["value"])

    def choose_color():
        result = colorchooser.askcolor(color=color_state["value"], title="Block colour")
        if result and result[1]:
            color_state["value"] = result[1]
            color_swatch.itemconfig("swatch", fill=result[1])
            redraw_preview()

    tk.Button(color_row, text="Choose...", command=choose_color).pack(side="left")

    for var in fields.values():
        var.trace_add("write", redraw_preview)
    redraw_preview()

    def cleanup_preview():
        if preview["id"] is not None:
            main_canvas.delete(preview["id"])
            preview["id"] = None

    error_label = tk.Label(dlg, text="", fg="#c0392b")
    error_label.pack(pady=(4, 0))

    def create():
        try:
            mass = float(fields["Mass (kg)"].get())
            w = float(fields["Width (m)"].get())
            h = float(fields["Height (m)"].get())
            px = float(fields["Position X (m)"].get())
            py = float(fields["Position Y (m)"].get())
            ang = float(fields["Angle (deg)"].get())
            if w <= 0 or h <= 0 or mass < 0:
                raise ValueError
        except ValueError:
            error_label.config(text="Mass must be >= 0, width/height must be > 0.")
            return

        new_block = Block(
            mass=mass, dimensions=(w, h),
            position=Vector2d(px, py), velocity=Vector2d(0, 0),
            forces=Vector2d(0, 0), angle=ang, color=color_state["value"]
        )
        new_block.name = f"block_{len(sim_objects)}"
        sim_objects.append(new_block)
        poly_id = main_canvas.create_polygon([0, 0, 0, 0, 0, 0, 0, 0],
                                              fill=color_state["value"], outline="black", width=2)
        canvas_ids[new_block] = poly_id
        refresh_block_list()
        select_block(new_block)
        cleanup_preview()
        dlg.destroy()

    tk.Button(dlg, text="Create", command=create).pack(pady=14)
    dlg.protocol("WM_DELETE_WINDOW", lambda: (cleanup_preview(), dlg.destroy()))


tk.Button(left_panel, text="+ Add block", command=open_add_block_dialog).pack(fill="x", padx=8, pady=(6, 8))

# ---------------------------------------------------------------------------
# Center: canvas + camera
# ---------------------------------------------------------------------------

main_canvas = tk.Canvas(root, bg="white")
main_canvas.place(x=LEFT_PANEL_WIDTH, y=TOP_BAR_HEIGHT, width=canvas_width, height=canvas_height)

camera = {"scale": 100.0, "offset_x": 0.0, "offset_y": 0.0}


def to_screen(v):
    sx = canvas_width / 2.0 + camera["offset_x"] + v.x * camera["scale"]
    sy = canvas_height / 2.0 + camera["offset_y"] - v.y * camera["scale"]
    return sx, sy


def screen_to_world(sx, sy):
    wx = (sx - canvas_width / 2.0 - camera["offset_x"]) / camera["scale"]
    wy = -(sy - canvas_height / 2.0 - camera["offset_y"]) / camera["scale"]
    return Vector2d(wx, wy)


# --- camera controls: right-drag to pan, scroll to zoom-to-cursor ---

pan_state = {"active": False, "last_x": 0, "last_y": 0}


def on_right_press(event):
    pan_state["active"] = True
    pan_state["last_x"] = event.x
    pan_state["last_y"] = event.y


def on_right_drag(event):
    if not pan_state["active"]:
        return
    dx = event.x - pan_state["last_x"]
    dy = event.y - pan_state["last_y"]
    camera["offset_x"] += dx
    camera["offset_y"] += dy
    pan_state["last_x"] = event.x
    pan_state["last_y"] = event.y


def on_right_release(event):
    pan_state["active"] = False


def on_scroll(event):
    if hasattr(event, "delta") and event.delta != 0:
        factor = 1.1 if event.delta > 0 else (1 / 1.1)
    elif getattr(event, "num", None) == 4:
        factor = 1.1
    elif getattr(event, "num", None) == 5:
        factor = 1 / 1.1
    else:
        return

    mouse_world = screen_to_world(event.x, event.y)
    camera["scale"] = max(10.0, min(400.0, camera["scale"] * factor))
    camera["offset_x"] = event.x - canvas_width / 2.0 - mouse_world.x * camera["scale"]
    camera["offset_y"] = event.y - canvas_height / 2.0 + mouse_world.y * camera["scale"]


main_canvas.bind("<ButtonPress-3>", on_right_press)
main_canvas.bind("<B3-Motion>", on_right_drag)
main_canvas.bind("<ButtonRelease-3>", on_right_release)
main_canvas.bind("<MouseWheel>", on_scroll)
main_canvas.bind("<Button-4>", on_scroll)
main_canvas.bind("<Button-5>", on_scroll)

# --- selection + drag-to-move (hard grab or soft spring pull) ---

selected_block = None
drag_state = {"block": None, "offset": None, "last_time": None}
dragging_spring = None
current_mouse_screen = (0, 0)


def get_drag_mouse_world_pos():
    return screen_to_world(current_mouse_screen[0], current_mouse_screen[1])


def select_block(block):
    global selected_block
    selected_block = block
    for b in sim_objects:
        b.selected = (b is block)
    if block is None:
        block_name_label.config(text="No block selected")
        block_info_label.config(text="")
    else:
        block_name_label.config(text=block.name)
        friction_static_slider.set(block.material.static_friction)
        friction_kinetic_slider.set(block.material.kinetic_friction)
        restitution_slider.set(block.material.restitution)
        mass_edit_var.set(f"{block.mass:.2f}")
        width_edit_var.set(f"{block.dimensions[0]:.2f}")
        height_edit_var.set(f"{block.dimensions[1]:.2f}")
    update_colour_swatch()
    refresh_force_list()
    refresh_block_list_highlight()


def on_left_press(event):
    global dragging_spring
    world_pt = screen_to_world(event.x, event.y)
    hit_block = None
    for b in reversed(sim_objects):
        if point_in_block(world_pt, b):
            hit_block = b
            break
    select_block(hit_block)

    if hit_block is None or hit_block is ground_block:
        return

    if drag_mode_var.get() == "soft":
        angle_rad = math.radians(hit_block.angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        world_offset = world_pt - hit_block.position
        local_x = world_offset.x * cos_a + world_offset.y * sin_a
        local_y = -world_offset.x * sin_a + world_offset.y * cos_a
        local_offset = Vector2d(local_x, local_y)
        dragging_spring = DragSpring(hit_block, local_offset, get_drag_mouse_world_pos)
        drag_state["block"] = hit_block
    else:
        drag_state["block"] = hit_block
        drag_state["offset"] = world_pt - hit_block.position
        drag_state["last_time"] = time.perf_counter()
        hit_block.stored_mass = hit_block.mass
        hit_block.mass = 0.0


def on_left_drag(event):
    global current_mouse_screen
    current_mouse_screen = (event.x, event.y)

    if drag_mode_var.get() == "hard":
        b = drag_state["block"]
        if b is None:
            return
        world_pt = screen_to_world(event.x, event.y)
        now = time.perf_counter()
        dt = max(now - drag_state["last_time"], 1e-4)
        new_pos = world_pt - drag_state["offset"]
        b.velocity = (new_pos - b.position) / dt
        b.position = new_pos
        drag_state["last_time"] = now
    # soft mode: DragSpring reads current_mouse_screen live each physics
    # substep via get_drag_mouse_world_pos, nothing to do here.


def on_mouse_move(event):
    global current_mouse_screen
    current_mouse_screen = (event.x, event.y)


def on_left_release(event):
    global dragging_spring
    b = drag_state["block"]
    if drag_mode_var.get() == "hard" and b is not None and b.stored_mass is not None:
        b.mass = b.stored_mass
        b.stored_mass = None
    dragging_spring = None
    drag_state["block"] = None


main_canvas.bind("<Button-1>", on_left_press)
main_canvas.bind("<B1-Motion>", on_left_drag)
main_canvas.bind("<ButtonRelease-1>", on_left_release)
main_canvas.bind("<Motion>", on_mouse_move)

# ---------------------------------------------------------------------------
# Right panel: tabbed inspector (Block / Global / Viz)
# ---------------------------------------------------------------------------

right_panel = tk.Frame(root, bg="#cfcfcf")
right_panel.place(x=WINDOW_WIDTH - RIGHT_PANEL_WIDTH, y=TOP_BAR_HEIGHT,
                   width=RIGHT_PANEL_WIDTH, height=WINDOW_HEIGHT - TOP_BAR_HEIGHT)

notebook = ttk.Notebook(right_panel)
notebook.pack(fill="both", expand=True, padx=5, pady=5)

block_tab = tk.Frame(notebook, bg="white")
global_tab = tk.Frame(notebook, bg="white")
viz_tab = tk.Frame(notebook, bg="white")
notebook.add(block_tab, text="Block")
notebook.add(global_tab, text="Global")
notebook.add(viz_tab, text="Viz")

# --- Block tab ---

block_name_label = tk.Label(block_tab, text="No block selected", bg="white", font=("Arial", 11, "bold"))
block_name_label.pack(anchor="w", padx=8, pady=(10, 4))

block_info_label = tk.Label(block_tab, text="", bg="white", justify="left", font=("Consolas", 9))
block_info_label.pack(anchor="w", padx=8, pady=4)

# Live-editable mass / dimensions
mass_edit_var = tk.StringVar(value="")
width_edit_var = tk.StringVar(value="")
height_edit_var = tk.StringVar(value="")


def apply_mass(*_):
    if selected_block is None:
        return
    try:
        m = float(mass_edit_var.get())
    except ValueError:
        return
    if m < 0:
        return
    selected_block.set_mass(m)


def apply_dimensions(*_):
    if selected_block is None:
        return
    try:
        w = float(width_edit_var.get())
        h = float(height_edit_var.get())
    except ValueError:
        return
    if w <= 0 or h <= 0:
        return
    selected_block.set_dimensions(w, h)


mass_row = tk.Frame(block_tab, bg="white")
mass_row.pack(fill="x", padx=8, pady=(8, 0))
tk.Label(mass_row, text="Mass (kg)", bg="white").pack(side="left")
tk.Entry(mass_row, textvariable=mass_edit_var, width=8).pack(side="left", padx=6)
tk.Button(mass_row, text="Apply", command=apply_mass).pack(side="left")

dims_row = tk.Frame(block_tab, bg="white")
dims_row.pack(fill="x", padx=8, pady=(6, 0))
tk.Label(dims_row, text="W x H (m)", bg="white").pack(side="left")
tk.Entry(dims_row, textvariable=width_edit_var, width=5).pack(side="left", padx=4)
tk.Entry(dims_row, textvariable=height_edit_var, width=5).pack(side="left", padx=4)
tk.Button(dims_row, text="Apply", command=apply_dimensions).pack(side="left")

colour_row = tk.Frame(block_tab, bg="white")
colour_row.pack(fill="x", padx=8, pady=(6, 0))
tk.Label(colour_row, text="Colour", bg="white").pack(side="left")
colour_swatch = tk.Canvas(colour_row, width=22, height=22, highlightthickness=1, highlightbackground="black")
colour_swatch.pack(side="left", padx=6)
colour_swatch.create_rectangle(0, 0, 22, 22, fill="white", outline="", tags="swatch")


def update_colour_swatch():
    color = selected_block.color if (selected_block and selected_block.color) else "#ffffff"
    colour_swatch.itemconfig("swatch", fill=color)


def choose_block_colour():
    if selected_block is None:
        return
    result = colorchooser.askcolor(color=selected_block.color or "#2980b9", title="Block colour")
    if result and result[1]:
        selected_block.color = result[1]
        main_canvas.itemconfig(canvas_ids[selected_block], fill=result[1])
        update_colour_swatch()


tk.Button(colour_row, text="Choose...", command=choose_block_colour).pack(side="left")

tk.Label(block_tab, text="Static friction", bg="white").pack(anchor="w", padx=8, pady=(10, 0))
friction_static_slider = tk.Scale(block_tab, from_=0, to=1.5, resolution=0.01, orient="horizontal",
                                   bg="white", highlightthickness=0)
friction_static_slider.pack(fill="x", padx=8)

tk.Label(block_tab, text="Kinetic friction", bg="white").pack(anchor="w", padx=8, pady=(6, 0))
friction_kinetic_slider = tk.Scale(block_tab, from_=0, to=1.5, resolution=0.01, orient="horizontal",
                                    bg="white", highlightthickness=0)
friction_kinetic_slider.pack(fill="x", padx=8)

tk.Label(block_tab, text="Restitution", bg="white").pack(anchor="w", padx=8, pady=(6, 0))
restitution_slider = tk.Scale(block_tab, from_=0, to=1, resolution=0.01, orient="horizontal",
                               bg="white", highlightthickness=0)
restitution_slider.pack(fill="x", padx=8)


def apply_material_from_sliders(*_):
    if selected_block is None:
        return
    selected_block.material.static_friction = friction_static_slider.get()
    selected_block.material.kinetic_friction = friction_kinetic_slider.get()
    selected_block.material.restitution = restitution_slider.get()


friction_static_slider.config(command=apply_material_from_sliders)
friction_kinetic_slider.config(command=apply_material_from_sliders)
restitution_slider.config(command=apply_material_from_sliders)

GRAPH_PROPERTIES = {
    "Position X (m)": "pos_x",
    "Position Y (m)": "pos_y",
    "Speed (m/s)": "vel",
    "Angle (deg)": "angle",
    "Angular velocity (deg/s)": "ang_vel",
    "Momentum (kg m/s)": "momentum",
}

GRAPH_WINDOW_OPTIONS = {
    "Last 10 s": 10.0,
    "Last 30 s": 30.0,
    "Last 60 s": 60.0,
    "Full history": None,
}


def open_graph_window():
    if selected_block is None:
        return
    block = selected_block
    win = tk.Toplevel(root)
    win.title(f"Graph - {block.name}")
    win.geometry("560x440")

    control_row = tk.Frame(win)
    control_row.pack(fill="x")
    tk.Label(control_row, text="Property:").pack(side="left", padx=6, pady=6)
    prop_var = tk.StringVar(value="Position Y (m)")
    tk.OptionMenu(control_row, prop_var, *GRAPH_PROPERTIES.keys()).pack(side="left")

    tk.Label(control_row, text="Window:").pack(side="left", padx=(12, 6))
    window_var = tk.StringVar(value="Last 30 s")
    tk.OptionMenu(control_row, window_var, *GRAPH_WINDOW_OPTIONS.keys()).pack(side="left")

    fig = plt.Figure(figsize=(5.5, 4), dpi=100)
    ax = fig.add_subplot(111)
    line, = ax.plot([], [])
    ax.set_xlabel("time (s)")
    canvas_widget = FigureCanvasTkAgg(fig, master=win)
    canvas_widget.get_tk_widget().pack(fill="both", expand=True)

    state = {"active": True}

    def on_close():
        state["active"] = False
        win.destroy()

    win.protocol("WM_DELETE_WINDOW", on_close)

    def refresh():
        if not state["active"]:
            return
        key = GRAPH_PROPERTIES[prop_var.get()]
        window_seconds = GRAPH_WINDOW_OPTIONS[window_var.get()]
        t_full = list(block.history["t"])
        y_full = list(block.history[key])

        if window_seconds is not None and t_full:
            cutoff = t_full[-1] - window_seconds
            t = [tv for tv in t_full if tv >= cutoff]
            y = y_full[-len(t):] if t else []
        else:
            t, y = t_full, y_full

        if t and y:
            line.set_data(t, y)
            ax.relim()
            ax.autoscale_view()
            ax.set_ylabel(prop_var.get())
            canvas_widget.draw()
        win.after(200, refresh)

    refresh()


tk.Button(block_tab, text="Graph this block", command=open_graph_window).pack(fill="x", padx=8, pady=(14, 4))


def open_add_force_dialog():
    if selected_block is None:
        return
    dlg = tk.Toplevel(root)
    dlg.title("Add constant force")
    dlg.geometry("240x220")

    tk.Label(dlg, text="Magnitude (N)").pack(anchor="w", padx=10, pady=(10, 0))
    mag_var = tk.StringVar(value="10")
    tk.Entry(dlg, textvariable=mag_var).pack(fill="x", padx=10)

    tk.Label(dlg, text="Angle (deg, 0 = up)").pack(anchor="w", padx=10, pady=(10, 0))
    angle_var = tk.StringVar(value="90")
    tk.Entry(dlg, textvariable=angle_var).pack(fill="x", padx=10)

    tk.Label(dlg, text="Name").pack(anchor="w", padx=10, pady=(10, 0))
    name_var = tk.StringVar(value=f"Push {len(selected_block.custom_forces) + 1}")
    tk.Entry(dlg, textvariable=name_var).pack(fill="x", padx=10)

    error_label = tk.Label(dlg, text="", fg="#c0392b")
    error_label.pack(pady=(4, 0))

    def create():
        try:
            mag = float(mag_var.get())
            ang = float(angle_var.get())
        except ValueError:
            error_label.config(text="Magnitude/angle must be numbers.")
            return
        vec = force_resolve(mag, ang)
        selected_block.custom_forces.append(ConstantForce(vec, name=name_var.get()))
        refresh_force_list()
        dlg.destroy()

    tk.Button(dlg, text="Add", command=create).pack(pady=14)


tk.Button(block_tab, text="Add constant force", command=open_add_force_dialog).pack(fill="x", padx=8, pady=4)

force_list_frame = tk.Frame(block_tab, bg="white")
force_list_frame.pack(fill="x", padx=8, pady=(4, 4))


def refresh_force_list():
    for widget in force_list_frame.winfo_children():
        widget.destroy()
    if selected_block is None:
        return
    for f in list(selected_block.custom_forces):
        row = tk.Frame(force_list_frame, bg="white")
        row.pack(fill="x", pady=2)
        tk.Label(row, text=f.name, bg="white").pack(side="left")
        tk.Button(row, text="x", width=2, command=lambda f=f: remove_force(f)).pack(side="right")


def remove_force(f):
    if selected_block and f in selected_block.custom_forces:
        selected_block.custom_forces.remove(f)
    refresh_force_list()


def delete_selected_block():
    if selected_block is not None:
        remove_block(selected_block)


tk.Button(block_tab, text="Delete this block", command=delete_selected_block).pack(fill="x", padx=8, pady=(10, 8))

# --- Global tab ---

tk.Label(global_tab, text="Gravity (m/s^2)", bg="white").pack(anchor="w", padx=8, pady=(12, 0))
gravity_var = tk.StringVar(value="9.81")
tk.Entry(global_tab, textvariable=gravity_var, width=8).pack(anchor="w", padx=8)

tk.Label(global_tab, text="Fluid density of air (kg/m^3)", bg="white").pack(anchor="w", padx=8, pady=(10, 0))
fluid_density_var = tk.StringVar(value="1.225")
tk.Entry(global_tab, textvariable=fluid_density_var, width=8).pack(anchor="w", padx=8)

tk.Label(global_tab, text="Engine substeps", bg="white").pack(anchor="w", padx=8, pady=(10, 0))
substeps_var = tk.StringVar(value="8")
tk.Entry(global_tab, textvariable=substeps_var, width=8).pack(anchor="w", padx=8)

tk.Label(global_tab, text="Velocity iterations", bg="white").pack(anchor="w", padx=8, pady=(10, 0))
iterations_var = tk.StringVar(value="16")
tk.Entry(global_tab, textvariable=iterations_var, width=8).pack(anchor="w", padx=8)

tk.Label(global_tab, text="Drag mode", bg="white", font=("Arial", 10, "bold")).pack(anchor="w", padx=8, pady=(16, 2))
drag_mode_var = tk.StringVar(value="hard")
tk.Radiobutton(global_tab, text="Hard grab (instant, held in place)", variable=drag_mode_var,
               value="hard", bg="white").pack(anchor="w", padx=8)
tk.Radiobutton(global_tab, text="Soft pull (spring, can be blocked)", variable=drag_mode_var,
               value="soft", bg="white").pack(anchor="w", padx=8)

# --- Viz tab ---

show_forces_var = tk.BooleanVar(value=True)
show_contacts_var = tk.BooleanVar(value=False)
show_axes_var = tk.BooleanVar(value=False)
show_penetration_var = tk.BooleanVar(value=False)

tk.Checkbutton(viz_tab, text="Force arrows", variable=show_forces_var, bg="white").pack(anchor="w", padx=8, pady=(14, 4))
tk.Checkbutton(viz_tab, text="Contact points", variable=show_contacts_var, bg="white").pack(anchor="w", padx=8, pady=4)
tk.Checkbutton(viz_tab, text="SAT axes", variable=show_axes_var, bg="white").pack(anchor="w", padx=8, pady=4)
tk.Checkbutton(viz_tab, text="Penetration vectors", variable=show_penetration_var, bg="white").pack(anchor="w", padx=8, pady=4)

legend_frame = tk.Frame(viz_tab, bg="white")
legend_frame.pack(anchor="w", padx=8, pady=(16, 4), fill="x")
tk.Label(legend_frame, text="Force colours", bg="white", font=("Arial", 9, "bold")).pack(anchor="w")


def add_legend_row(text, color):
    row = tk.Frame(legend_frame, bg="white")
    row.pack(anchor="w", pady=1)
    sw = tk.Canvas(row, width=14, height=14, bg="white", highlightthickness=0)
    sw.pack(side="left")
    sw.create_rectangle(1, 1, 13, 13, fill=color, outline="")
    tk.Label(row, text=text, bg="white").pack(side="left", padx=4)


add_legend_row("Gravity", FORCE_COLORS["Gravity"])
add_legend_row("Drag", FORCE_COLORS["Drag"])
add_legend_row("Friction", FORCE_COLORS["Friction"])
add_legend_row("Soft pull", FORCE_COLORS["Pull"])
add_legend_row("Custom / constant force", CUSTOM_FORCE_COLOR)

# ---------------------------------------------------------------------------
# Simulation setup
# ---------------------------------------------------------------------------

environmental_forces = [Gravity(g=9.81), Drag(fluid_density=1.225)]

b1 = Block(mass=25.0, dimensions=(3.0, 0.8), position=Vector2d(0.5, -2.5), velocity=Vector2d(0, 0), forces=Vector2d(0, 0), angle=0.0)
b2 = Block(mass=18.0, dimensions=(2.4, 0.7), position=Vector2d(0.6, -1.5), velocity=Vector2d(0, 0), forces=Vector2d(0, 0), angle=2.0)
b3 = Block(mass=15.0, dimensions=(1.8, 0.8), position=Vector2d(-0.46, -0.5), velocity=Vector2d(0, 0), forces=Vector2d(0, 0), angle=-3.0)
b4 = Block(mass=12.0, dimensions=(2.0, 0.6), position=Vector2d(2.05, 0.5), velocity=Vector2d(0, 0), forces=Vector2d(0, 0), angle=1.5)
b5 = Block(mass=10.0, dimensions=(1.2, 0.8), position=Vector2d(-0.08, 1.5), velocity=Vector2d(0, 0), forces=Vector2d(0, 0), angle=-1.0)
b6 = Block(mass=8.0, dimensions=(2.0, 0.5), position=Vector2d(0.0, 2.5), velocity=Vector2d(0, 0), forces=Vector2d(0, 0), angle=4.0)
b7 = Block(mass=1000.0, dimensions=(1, 1), position=Vector2d(0.7, 3.8), velocity=Vector2d(0, 0), forces=Vector2d(0, 0), angle=-0.0)

ground_block = Block(
    mass=0.0,
    dimensions=(12.0, 1.0),
    position=Vector2d(0.0, -3.5),
    velocity=Vector2d(0.0, 0.0),
    forces=Vector2d(0, 0),
    angle=0.0,
    angular_velocity=0.0
)

sim_objects = [b1, b2, b3, b4, b5, b6, b7, ground_block]
for i, b in enumerate(sim_objects):
    b.name = "ground" if b is ground_block else f"block_{i + 1}"

canvas_ids = {}
tower_colors = [
    "#2c3e50", "#34495e", "#16a085", "#27ae60",
    "#2980b9", "#8e44ad", "#f39c12", "#7f8c8d"
]

for i, obj in enumerate(sim_objects):
    fill_color = "#333333" if obj.mass == 0.0 else tower_colors[i % len(tower_colors)]
    obj.color = fill_color
    poly_id = main_canvas.create_polygon([0, 0, 0, 0, 0, 0, 0, 0], fill=fill_color, outline="black", width=2)
    canvas_ids[obj] = poly_id

debug_collision_info = {"pairs": []}

# ---------------------------------------------------------------------------
# Rendering helpers
# ---------------------------------------------------------------------------


def render_objects():
    for obj in sim_objects:
        world_corners = obj.calculate_corners()
        pixel_array = []
        for corner in world_corners:
            sx, sy = to_screen(corner)
            pixel_array.extend([sx, sy])
        tk_id = canvas_ids[obj]
        main_canvas.coords(tk_id, *pixel_array)
        outline = "#e67e22" if obj.selected else "black"
        width = 3 if obj.selected else 2
        main_canvas.itemconfig(tk_id, outline=outline, width=width)


def draw_force_arrows():
    main_canvas.delete("force_arrow")
    if not show_forces_var.get():
        return
    for b in sim_objects:
        if b.mass == 0.0:
            continue
        sx, sy = to_screen(b.position)
        for name, vec in b.force_breakdown.items():
            mag = vec.magnitude()
            if mag < 1e-6:
                continue
            direction = vec.normalise()
            length = min(80, 10 + mag * 0.3)
            ex = sx + direction.x * length
            ey = sy - direction.y * length
            color = get_force_color(name)
            main_canvas.create_line(sx, sy, ex, ey, fill=color, width=2, arrow=tk.LAST, tags="force_arrow")


def draw_collision_debug():
    main_canvas.delete("collision_debug")
    if not (show_contacts_var.get() or show_axes_var.get() or show_penetration_var.get()):
        return
    for pair in debug_collision_info.get("pairs", []):
        obj1, obj2 = pair["obj1"], pair["obj2"]
        normal = pair["normal"]
        penetration = pair["penetration"]

        if show_axes_var.get():
            for axis in pair["axes"]:
                for obj in (obj1, obj2):
                    sx, sy = to_screen(obj.position)
                    ex = sx + axis.x * 40
                    ey = sy - axis.y * 40
                    main_canvas.create_line(sx, sy, ex, ey, fill="#f39c12", width=1, dash=(3, 2), tags="collision_debug")

        if show_contacts_var.get():
            for cp in pair["contact_points"]:
                sx, sy = to_screen(cp)
                main_canvas.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill="#e74c3c", outline="", tags="collision_debug")

        if show_penetration_var.get():
            for cp in pair["contact_points"]:
                sx, sy = to_screen(cp)
                ex = sx + normal.x * penetration * camera["scale"]
                ey = sy - normal.y * penetration * camera["scale"]
                main_canvas.create_line(sx, sy, ex, ey, fill="#d63384", width=2, arrow=tk.LAST, tags="collision_debug")


def update_block_info_label():
    if selected_block is None:
        return
    b = selected_block
    text = (
        f"Position: {b.position.x:.2f}, {b.position.y:.2f} m\n"
        f"Velocity: {b.velocity.x:.2f}, {b.velocity.y:.2f} m/s\n"
        f"Speed: {b.velocity.magnitude():.2f} m/s\n"
        f"Angle: {b.angle:.1f} deg\n"
        f"Angular vel: {b.angular_velocity:.1f} deg/s\n"
        f"Mass: {b.mass:.2f} kg\n"
        f"Momentum: {b.momentum.magnitude():.2f} kg m/s"
    )
    block_info_label.config(text=text)


refresh_block_list()
update_colour_swatch()

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def animate():
    global playback_index

    try:
        environmental_forces[0].g = float(gravity_var.get())
    except ValueError:
        pass

    try:
        environmental_forces[1].rho = float(fluid_density_var.get())
    except ValueError:
        pass

    try:
        substeps = max(1, int(substeps_var.get()))
    except ValueError:
        substeps = 8

    try:
        velocity_iterations = max(1, int(iterations_var.get()))
    except ValueError:
        velocity_iterations = 16

    if is_playing_back and recording_buffer:
        idx = min(playback_index, len(recording_buffer) - 1)
        frame = recording_buffer[idx]
        for obj in sim_objects:
            if obj.name in frame:
                pos, angle, vel, ang_vel = frame[obj.name]
                obj.position = pos
                obj.angle = angle
                obj.velocity = vel
                obj.angular_velocity = ang_vel
        playback_index = min(playback_index + 1, len(recording_buffer) - 1)
        scrub_var.set(playback_index)
        main_canvas.delete("force_arrow")
        main_canvas.delete("collision_debug")
    elif not is_paused:
        active_forces = list(environmental_forces)
        if dragging_spring is not None:
            active_forces.append(dragging_spring)

        world_step(sim_objects, SIM_FPS, active_forces,
                   velocity_iterations=velocity_iterations, substeps=substeps,
                   debug_out=debug_collision_info)

        if is_recording and recording_file is not None:
            write_recording_frame(recording_file, sim_objects)

        draw_force_arrows()
        draw_collision_debug()

    render_objects()
    update_block_info_label()
    refresh_block_list_highlight()

    root.after(int(1000 / SIM_FPS), animate)


animate()
root.mainloop()