import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as PlotCircle
from matplotlib.patches import Polygon


class Circle:
    def __init__(self, name, x, y, radius):
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.radius = float(radius)
        self.colliding = False

class Rectangle:
    def __init__(self, name, x, y, width, height, angle):
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.width = float(width)
        self.height = float(height)
        self.angle = float(angle)
        self.colliding = False

    def get_corners(self):
        """
        Returns the 4 rotated corners of rectangle
        Rotation center = top-left corner
        """

        angle_rad = math.radians(self.angle)

        corners = [
            (0, 0),
            (self.width, 0),
            (self.width, self.height),
            (0, self.height)
        ]

        rotated = []

        for px, py in corners:
            rx = px * math.cos(angle_rad) - py * math.sin(angle_rad)
            ry = px * math.sin(angle_rad) + py * math.cos(angle_rad)

            rotated.append((rx + self.x, ry + self.y))

        return rotated


# file reading

def read_objects(filename):
    objects = []

    with open(filename, "r") as file:
        for line in file:

            line = line.strip()

            # skip comments and empty lines
            if not line or line.startswith("!"):
                continue

            parts = line.split()

            if parts[0].lower() == "circle":
                _, name, x, y, radius = parts
                objects.append(Circle(name, x, y, radius))

            elif parts[0].lower() == "rectangle":
                _, name, x, y, w, h, angle = parts
                objects.append(Rectangle(name, x, y, w, h, angle))

    return objects


# collision

def circle_vs_circle(c1, c2):

    dx = c1.x - c2.x
    dy = c1.y - c2.y

    distance = math.sqrt(dx * dx + dy * dy)

    return distance <= (c1.radius + c2.radius)


# helpers

def normalize(v):
    length = math.sqrt(v[0]**2 + v[1]**2)

    if length == 0:
        return (0,0)

    return (v[0] / length, v[1] / length)


def project_polygon(axis, points):

    dots = [point[0] * axis[0] + point[1] * axis[1]
            for point in points]

    return min(dots), max(dots)


def overlap(proj1, proj2):
    return proj1[0] <= proj2[1] and proj2[0] <= proj1[1]


# rectangle vs rectangle
# using sat

def rect_vs_rect(r1, r2):

    corners1 = r1.get_corners()
    corners2 = r2.get_corners()

    axes = []

    # get normals from r1
    for i in range(4):
        p1 = corners1[i]
        p2 = corners1[(i + 1) % 4]

        edge = (p2[0] - p1[0], p2[1] - p1[1])

        normal = normalize((-edge[1], edge[0]))

        axes.append(normal)

    # get normals from r2
    for i in range(4):
        p1 = corners2[i]
        p2 = corners2[(i + 1) % 4]

        edge = (p2[0] - p1[0], p2[1] - p1[1])

        normal = normalize((-edge[1], edge[0]))

        axes.append(normal)

    # SAT test
    for axis in axes:

        proj1 = project_polygon(axis, corners1)
        proj2 = project_polygon(axis, corners2)

        if not overlap(proj1, proj2):
            return False

    return True


# circle vs rectangle

def circle_vs_rect(circle, rect):

    angle = math.radians(-rect.angle)

    # translate circle into rectangle local space
    tx = circle.x - rect.x
    ty = circle.y - rect.y

    local_x = tx * math.cos(angle) - ty * math.sin(angle)
    local_y = tx * math.sin(angle) + ty * math.cos(angle)

    # closest point
    closest_x = max(0, min(local_x, rect.width))
    closest_y = max(0, min(local_y, rect.height))

    dx = local_x - closest_x
    dy = local_y - closest_y

    return (dx * dx + dy * dy) <= (circle.radius ** 2)


# main checker

def check_collisions(objects):

    collisions = []

    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):

            a = objects[i]
            b = objects[j]

            collided = False

            # circle vs circle
            if isinstance(a, Circle) and isinstance(b, Circle):
                collided = circle_vs_circle(a, b)

            # rectangle vs rectangle
            elif isinstance(a, Rectangle) and isinstance(b, Rectangle):
                collided = rect_vs_rect(a, b)

            # circle vs rectangle
            elif isinstance(a, Circle) and isinstance(b, Rectangle):
                collided = circle_vs_rect(a, b)

            elif isinstance(a, Rectangle) and isinstance(b, Circle):
                collided = circle_vs_rect(b, a)

            if collided:
                a.colliding = True
                b.colliding = True

                collisions.append((a.name, b.name))

    return collisions


# visualize

def draw_objects(objects):

    fig, ax = plt.subplots(figsize=(12, 10))

    for obj in objects:

        color = "red" if obj.colliding else "blue"

        # circle
        if isinstance(obj, Circle):

            circle = PlotCircle(
                (obj.x, obj.y),
                obj.radius,
                color=color,
                alpha=0.5
            )

            ax.add_patch(circle)

            ax.text(obj.x, obj.y + obj.radius + 5,
                    obj.name,
                    ha='center')

        # rectangle
        elif isinstance(obj, Rectangle):

            corners = obj.get_corners()

            poly = Polygon(
                corners,
                closed=True,
                color=color,
                alpha=0.5
            )

            ax.add_patch(poly)

            ax.text(obj.x + 5, obj.y + 5,
                    obj.name)

    ax.set_xlim(0, 850)
    ax.set_ylim(0, 800)

    ax.set_aspect('equal')

    plt.title("Collision Visualization")

    plt.xlabel("X")
    plt.ylabel("Y")

    plt.grid(True)

    plt.show()


def main():

    filename = "objects.txt"

    objects = read_objects(filename)

    collisions = check_collisions(objects)

    print("Collisions found:\n")

    for a, b in collisions:
        print(f"{a} collides with {b}")

    draw_objects(objects)


if __name__ == "__main__":
    main()