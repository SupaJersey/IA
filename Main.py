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
    minimum_translation_vector_to_resolve = None

    corners1 = block1.calculate_corners()
    corners2 = block2.calculate_corners()

    for axis in axes:
        min1, max1 = project_corners(corners1, axis)
        min2, max2 = project_corners(corners2, axis)

        overlap = overlap_check(min1, max1, min2, max2)
        if overlap <= 0:
            return False, None, None
        else:
            if overlap < min_overlap:
                min_overlap = overlap
                minimum_translation_vector_to_resolve = axis

    return True, minimum_translation_vector_to_resolve, min_overlap

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

    def calculate(self, block, drag_coefficient):
        velocity = block.velocity.magnitude()
        if velocity == 0:
            return Vector2d(0,0)
        area = block.dimensions[1] * block.dimensions[2]

        drag_magnitude = 0.5 * self.rho * drag_coefficient * area * velocity**2

        drag_direction = block.velocity.normalise() * -1

        return drag_direction * drag_magnitude

class Local_Forces:
    @staticmethod
    def calculate_friction(object1, object2,contact_normal):
        mu1 = object1.friction
        mu2 = object2.friction
        mu = math.sqrt(mu1 * mu2)

        relative_velocity = object1.velocity - object2.velocity
        tangent = Vector2d(-contact_normal.y, contact_normal.x).normalise()

        velocity_along_tangent = relative_velocity.dot(tangent)
        if velocity_along_tangent > 0:
            friction_direction = tangent * -1
        else:
            friction_direction = tangent

        magnitude_normal_force = max(0, (object1.forces - object2.forces).dot(contact_normal))

        magnitude_friction = mu * magnitude_normal_force

        return(friction_direction * magnitude_friction)



class Block:


    def __init__(self, mass, dimensions, position, velocity, forces, angle):
        self.mass = mass
        self.dimensions = dimensions
        self.position = Vector2d(position[0] , position[1])
        self.velocity = Vector2d(velocity[0], velocity[1])
        self.forces = Vector2d(forces[0], forces[1])
        self.angle = angle

        self.past_velocity = []

    @property
    def volume(self):
        return self.dimensions[0] * self.dimensions[1] * self.dimensions[2]

    @property
    def density(self):
        return self.mass / self.volume

    @property
    def acceleration(self):
        return self.forces/self.mass

    @property
    def momentum(self):
        return self.velocity * self.mass

    def calculate_corners(self):
        half_width = self.dimensions[0] / 2
        half_height = self.dimensions[1] / 2

        relative_corners = [Vector2d(-half_width, -half_height), Vector2d(half_width, -half_height), Vector2d(half_width, half_height), Vector2d(-half_width, half_height)]

        block_angle_radians = math.radians(self.angle)
        cos_angle = math.cos(block_angle_radians)
        sin_angle = math.sin(block_angle_radians)

        world_corners = []

        for corner in relative_corners:
            rotated_corner = Vector2d(corner.x * cos_angle - corner.y *sin_angle, corner.x * sin_angle + corner.y * cos_angle)
            world_corners.append(self.position + rotated_corner)

        return world_corners


    def __str__(self):
        return f'A Block of mass {self.mass}, dimensions {self.dimensions[0]} by {self.dimensions[1]} by {self.dimensions[2]} located x:{self.position[0]} y:{self.position[1]} moving x:{self.velocity[0]}/s y:{self.velocity[1]}/s'

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
        pass



print(force_resolve(1,0, False))
print(force_resolve(1,45, False))
print(force_resolve(1,90, False))
print(force_resolve(1,180, False))
print(force_resolve(1,270, False))

