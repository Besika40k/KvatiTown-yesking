import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as PlotCircle
from matplotlib.patches import Polygon


# =========================
# SHAPE CLASSES
# =========================

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
        Returns the 4 rotated corners of the rectangle.
        Rotation center is the top-left corner.
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


# =========================
# FILE READING
# =========================

def read_objects(filename):
    objects = []

    with open(filename, "r") as file:
        for line in file:
            line = line.strip()

            # Skip empty lines and comments
            if not line or line.startswith("!"):
                continue

            parts = line.split()

            if parts[0].lower() == "circle":
                _, name, x, y, radius = parts
                objects.append(Circle(name, x, y, radius))

            elif parts[0].lower() == "rectangle":
                _, name, x, y, width, height, angle = parts
                objects.append(Rectangle(name, x, y, width, height, angle))

    return objects


# =========================
# COLLISION FUNCTIONS
# =========================

def circle_vs_circle(c1, c2):
    dx = c1.x - c2.x
    dy = c1.y - c2.y

    distance_squared = dx * dx + dy * dy
    radius_sum = c1.radius + c2.radius

    return distance_squared <= radius_sum * radius_sum


# =========================
# SAT HELPERS
# =========================

def normalize(vector):
    x, y = vector
    length = math.sqrt(x * x + y * y)

    if length == 0:
        return (0, 0)

    return (x / length, y / length)


def project_polygon(axis, points):
    dots = []

    for point in points:
        dot = point[0] * axis[0] + point[1] * axis[1]
        dots.append(dot)

    return min(dots), max(dots)


def overlap(projection1, projection2):
    return projection1[0] <= projection2[1] and projection2[0] <= projection1[1]


# =========================
# RECTANGLE VS RECTANGLE
# USING SAT
# =========================

def rect_vs_rect(r1, r2):
    corners1 = r1.get_corners()
    corners2 = r2.get_corners()

    axes = []

    # Axes from first rectangle
    for i in range(4):
        p1 = corners1[i]
        p2 = corners1[(i + 1) % 4]

        edge = (p2[0] - p1[0], p2[1] - p1[1])
        normal = normalize((-edge[1], edge[0]))

        axes.append(normal)

    # Axes from second rectangle
    for i in range(4):
        p1 = corners2[i]
        p2 = corners2[(i + 1) % 4]

        edge = (p2[0] - p1[0], p2[1] - p1[1])
        normal = normalize((-edge[1], edge[0]))

        axes.append(normal)

    # SAT test
    for axis in axes:
        projection1 = project_polygon(axis, corners1)
        projection2 = project_polygon(axis, corners2)

        if not overlap(projection1, projection2):
            return False

    return True


# =========================
# CIRCLE VS RECTANGLE
# WORKS FOR ROTATED RECTANGLES
# =========================

def circle_vs_rect(circle, rect):
    # Reverse the rectangle rotation
    angle_rad = math.radians(-rect.angle)

    # Move circle center relative to rectangle top-left corner
    tx = circle.x - rect.x
    ty = circle.y - rect.y

    # Rotate circle center into rectangle's local coordinate system
    local_x = tx * math.cos(angle_rad) - ty * math.sin(angle_rad)
    local_y = tx * math.sin(angle_rad) + ty * math.cos(angle_rad)

    # Find closest point inside the rectangle
    closest_x = max(0, min(local_x, rect.width))
    closest_y = max(0, min(local_y, rect.height))

    dx = local_x - closest_x
    dy = local_y - closest_y

    return dx * dx + dy * dy <= circle.radius * circle.radius


# =========================
# MAIN COLLISION CHECKER
# =========================

def check_collisions(objects):
    collisions = []

    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):
            a = objects[i]
            b = objects[j]

            collided = False

            if isinstance(a, Circle) and isinstance(b, Circle):
                collided = circle_vs_circle(a, b)

            elif isinstance(a, Rectangle) and isinstance(b, Rectangle):
                collided = rect_vs_rect(a, b)

            elif isinstance(a, Circle) and isinstance(b, Rectangle):
                collided = circle_vs_rect(a, b)

            elif isinstance(a, Rectangle) and isinstance(b, Circle):
                collided = circle_vs_rect(b, a)

            if collided:
                a.colliding = True
                b.colliding = True
                collisions.append((a.name, b.name))

    return collisions


# =========================
# DRAWING
# =========================

def draw_objects(objects):
    fig, ax = plt.subplots(figsize=(12, 10))

    for obj in objects:
        color = "red" if obj.colliding else "blue"

        if isinstance(obj, Circle):
            circle = PlotCircle(
                (obj.x, obj.y),
                obj.radius,
                color=color,
                alpha=0.5
            )

            ax.add_patch(circle)

            ax.text(
                obj.x,
                obj.y + obj.radius + 5,
                obj.name,
                ha="center"
            )

        elif isinstance(obj, Rectangle):
            corners = obj.get_corners()

            polygon = Polygon(
                corners,
                closed=True,
                color=color,
                alpha=0.5
            )

            ax.add_patch(polygon)

            ax.text(
                obj.x + 5,
                obj.y + 5,
                obj.name
            )

    ax.set_xlim(0, 850)
    ax.set_ylim(0, 800)

    ax.set_aspect("equal")

    plt.title("Collision Visualization")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)

    plt.show()


# =========================
# MAIN
# =========================

def main():
    filename = "objects.txt"

    objects = read_objects(filename)

    collisions = check_collisions(objects)

    draw_objects(objects)

    print("COLLISIONS FOUND:\n")

    if not collisions:
        print("No collisions found.")
    else:
        for a, b in collisions:
            print(f"{a} collides with {b}")


if __name__ == "__main__":
    main()