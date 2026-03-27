import math

#Global Settings

frame_rate = 60
gravitational_acceleration = 9.81


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
            return self.x * other.x + self.y * other.y

        else:
            print("Vector Multiply Error")

    __rmul__ = __mul__

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Vector2d(self.x / other, self.y / other)

        elif isinstance(other, (list, tuple)) and len(other) == 2:
            return Vector2d(self.x / other[0], self.y / other[1])

        elif isinstance(other, Vector2d):
            return self.x / other.x + self.y / other.y

        else:
            print("Vector Division Error")

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


class Gravity():
    def __init__(self, g=9.81):
        self.g = g

    def apply(self, block):
        return Vector2d (0, block.mass * -self.g)

class Friction():
    def __init__(self, mu=0):
        self.mu = mu

    def apply(self, block):
        if block.velocity.magnitude == 0:
            return Vector2d(0, 0)


class Block:

    def __init__(self, mass, dimensions, position, velocity, forces, angular_velocity):
        self.mass = mass
        self.dimensions = dimensions
        self.position = Vector2d(position)
        self.velocity = Vector2d(velocity)
        self.forces = Vector2d(forces)

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
        self.forces = Vector2d(0,0)
        for force in external_forces:
            self.forces = self.forces + force_resolve(force[0], force[1])
        self.velocity = (self.velocity + self.acceleration) * 1/frame_rate
        self.position = (self.position + self.velocity) * 1/frame_rate




print(force_resolve(1,0, False))
print(force_resolve(1,45, False))
print(force_resolve(1,90, False))
print(force_resolve(1,180, False))
print(force_resolve(1,270, False))

