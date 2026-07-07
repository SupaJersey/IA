import math

#Global Settings

frame_rate = 60
gravitational_acceleration = 9.81
fluid_density = 1.225

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
            return Vector2d(0,0)
        return self / magnitude

    def dot(self,other):
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




def force_resolve(magnitude, angle, in_radians = False):
    if in_radians == False:
        angle = math.radians(angle)
    return Vector2d(magnitude * math.sin(angle), magnitude * math.cos(angle))

def calculate_axes_from_corners(corners):
    axes = []

    for i in range(4):
        point1 = corners[i]
        point2 = corners[(i + 1) % 4]

        edge = point2 - point1

        axis = Vector2d(-edge.y,edge.x).normalise()
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
    return min(max1,max2) - max(min1,min2)

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

    # --- FIX: enforce consistent normal direction ---
    center_delta = block2.position - block1.position

    if center_delta.dot(best_axis) < 0:
        best_axis = -best_axis

    return True, best_axis, min_overlap

class Collision_Detection:
    pass

class Gravity:
    def __init__(self, g=9.81):
        self.g = g

    def calculate(self, block):
        return Vector2d (0, block.mass * -self.g)

class Drag:
    def __init__(self, fluid_density=1.225):
        self.rho = fluid_density

    def calculate(self, block, drag_coefficient=0.47):
        velocity = block.velocity.magnitude()
        if velocity == 0:
            return Vector2d(0,0)
        area = block.dimensions[0] * block.dimensions[1]
        drag_magnitude = 0.5 * self.rho * drag_coefficient * area * velocity**2
        drag_direction = block.velocity.normalise() * -1
        return drag_direction * drag_magnitude


def apply_collision_impulse(obj1, obj2, normal, penetration, restitution=0.1):
    corners1 = obj1.calculate_corners()
    corners2 = obj2.calculate_corners()

    contact_point = (obj1.position + obj2.position) * 0.5

    r1 = contact_point - obj1.position
    r2 = contact_point - obj2.position

    v1 = obj1.velocity + Vector2d.rcross(None, obj1.angular_velocity, r1)
    v2 = obj2.velocity + Vector2d.rcross(None, obj2.angular_velocity, r2)

    relative_velocity = v1 - v2
    vel_along_normal = relative_velocity.dot(normal)

    if vel_along_normal > 0:
        return

    r1_cross_n = r1.cross(normal)
    r2_cross_n = r2.cross(normal)

    denom = (1 / obj1.mass) + (1 / obj2.mass) + \
            (r1_cross_n ** 2 / obj1.inertia) + (r2_cross_n ** 2 / obj2.inertia)

    j = -(1 + restitution) * vel_along_normal
    j /= denom

    impulse = normal * j

    obj1.velocity = obj1.velocity + impulse / obj1.mass
    obj2.velocity = obj2.velocity - impulse / obj2.mass

    obj1.angular_velocity = obj1.angular_velocity + r1.cross(impulse) / obj1.inertia
    obj2.angular_velocity = obj2.angular_velocity - r2.cross(impulse) / obj2.inertia


class Block:
    def __init__(self, mass, dimensions, position, velocity, forces, angle, angular_velocity=0.0):
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

    @property
    def volume(self):
        return self.dimensions[0] * self.dimensions[1]

    @property
    def density(self):
        return self.mass / self.volume

    @property
    def acceleration(self):
        return self.forces / self.mass

    @property
    def angular_acceleration(self):
        if self.inertia == 0:
            return 0.0
        return self.torque / self.inertia

    @property
    def momentum(self):
        return self.velocity * self.mass

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

    def update_position(self, frame_rate, external_forces):
        dt = 1 / frame_rate
        self.forces = Vector2d(0, 0)
        self.torque = 0.0

        for force in external_forces:
            self.forces = self.forces + force.calculate(self)

        self.velocity = self.velocity + self.acceleration * dt
        self.position = self.position + self.velocity * dt

        self.angular_velocity = self.angular_velocity + self.angular_acceleration * dt
        self.angle = self.angle + math.degrees(self.angular_velocity * dt)


    def __str__(self):
        return f'A Block of mass {self.mass}, dimensions {self.dimensions[0]} by {self.dimensions[1]} by {self.dimensions[2]} located x:{self.position.x} y:{self.position.y} moving x:{self.velocity.x}/s y:{self.velocity.y}/s'

    def advanced_print(self):
        print("mass:",self.mass)
        print("dimensions:",self.dimensions)
        print("position:",self.position)
        print("velocity:",self.velocity)
        print("forces:",self.forces)
        print("past_velocity:",self.past_velocity)
        print("volume:",self.volume)
        print("density:",self.density)
        print("acceleration:",self.acceleration)
        print("momentum:",self.momentum)

    def update_position(self, frame_rate, external_forces):
        dt = 1 / frame_rate
        self.forces = Vector2d(0, 0)
        for force in external_forces:
            self.forces = self.forces + force.calculate(self)
        self.velocity = self.velocity + self.acceleration * dt
        self.position = self.position + self.velocity * dt

def world_step(objects, frame_rate, external_forces):

    dt = 1 / frame_rate

    for obj in objects:
        obj.update_position(frame_rate, external_forces)

    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):

            obj1 = objects[i]
            obj2 = objects[j]

            hit, normal, penetration = check_collision(obj1, obj2)

            if not hit:
                continue

            resolve_penetration(obj1, obj2, normal, penetration)

            apply_collision_impulse(obj1, obj2, normal, restitution=0.1)

            apply_friction_impulse(obj1, obj2, normal)


