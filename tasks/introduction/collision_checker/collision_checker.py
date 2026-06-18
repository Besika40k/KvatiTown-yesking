import math
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as CirclePatch, Polygon


class Circle:
    def __init__(self, name, x, y, r):
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.r = float(r)


class Rectangle:
    def __init__(self, name, x, y, w, h, angle):
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.w = float(w)
        self.h = float(h)
        self.angle_degrees = float(angle)
        self.angle = math.radians(float(angle))

    def corners(self):
        points = [
            (0, 0),
            (self.w, 0),
            (self.w, self.h),
            (0, self.h)
        ]

        rotated_points = []

        for px, py in points:
            rotated_x = px * math.cos(self.angle) - py * math.sin(self.angle)
            rotated_y = px * math.sin(self.angle) + py * math.cos(self.angle)

            rotated_points.append(
                (self.x + rotated_x, self.y + rotated_y)
            )

        return rotated_points


def read_objects(filename):
    objects = []

    with open(filename, "r") as file:
        for line in file:
            line = line.strip()

            if not line:
                continue

            if line.startswith("!"):
                continue

            parts = line.split()

            if parts[0] == "circle":
                objects.append(
                    Circle(
                        name=parts[1],
                        x=parts[2],
                        y=parts[3],
                        r=parts[4]
                    )
                )

            elif parts[0] == "rectangle":
                objects.append(
                    Rectangle(
                        name=parts[1],
                        x=parts[2],
                        y=parts[3],
                        w=parts[4],
                        h=parts[5],
                        angle=parts[6]
                    )
                )

    return objects


def circle_circle_collision(c1, c2):
    dx = c1.x - c2.x
    dy = c1.y - c2.y

    distance_squared = dx * dx + dy * dy
    radius_sum = c1.r + c2.r

    return distance_squared <= radius_sum * radius_sum


def get_axes(corners):
    axes = []

    for i in range(len(corners)):
        p1 = corners[i]
        p2 = corners[(i + 1) % len(corners)]

        edge_x = p2[0] - p1[0]
        edge_y = p2[1] - p1[1]

        normal_x = -edge_y
        normal_y = edge_x

        length = math.sqrt(normal_x * normal_x + normal_y * normal_y)

        if length != 0:
            axes.append((normal_x / length, normal_y / length))

    return axes


def project_points(points, axis):
    projections = []

    for x, y in points:
        projection = x * axis[0] + y * axis[1]
        projections.append(projection)

    return min(projections), max(projections)


def rectangle_rectangle_collision(r1, r2):
    r1_corners = r1.corners()
    r2_corners = r2.corners()

    axes = get_axes(r1_corners) + get_axes(r2_corners)

    for axis in axes:
        min1, max1 = project_points(r1_corners, axis)
        min2, max2 = project_points(r2_corners, axis)

        if max1 < min2 or max2 < min1:
            return False

    return True


def circle_rectangle_collision(circle, rect):
    # Move circle center relative to rectangle top-left corner
    dx = circle.x - rect.x
    dy = circle.y - rect.y

    # Rotate circle center into rectangle's local coordinate system
    cos_a = math.cos(-rect.angle)
    sin_a = math.sin(-rect.angle)

    local_x = dx * cos_a - dy * sin_a
    local_y = dx * sin_a + dy * cos_a

    # Find closest point inside rectangle to circle center
    closest_x = max(0, min(local_x, rect.w))
    closest_y = max(0, min(local_y, rect.h))

    distance_x = local_x - closest_x
    distance_y = local_y - closest_y

    return distance_x * distance_x + distance_y * distance_y <= circle.r * circle.r


def objects_collide(obj1, obj2):
    if isinstance(obj1, Circle) and isinstance(obj2, Circle):
        return circle_circle_collision(obj1, obj2)

    if isinstance(obj1, Rectangle) and isinstance(obj2, Rectangle):
        return rectangle_rectangle_collision(obj1, obj2)

    if isinstance(obj1, Circle) and isinstance(obj2, Rectangle):
        return circle_rectangle_collision(obj1, obj2)

    if isinstance(obj1, Rectangle) and isinstance(obj2, Circle):
        return circle_rectangle_collision(obj2, obj1)

    return False


def find_collisions(objects):
    collisions = []
    colliding_objects = set()

    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):
            obj1 = objects[i]
            obj2 = objects[j]

            if objects_collide(obj1, obj2):
                collisions.append((obj1.name, obj2.name))
                colliding_objects.add(obj1.name)
                colliding_objects.add(obj2.name)

    return collisions, colliding_objects


def draw_objects(objects, colliding_objects):
    fig, ax = plt.subplots(figsize=(10, 9))

    for obj in objects:
        is_colliding = obj.name in colliding_objects

        if isinstance(obj, Circle):
            color = "red" if is_colliding else "blue"

            circle_patch = CirclePatch(
                (obj.x, obj.y),
                obj.r,
                alpha=0.6,
                color=color
            )

            ax.add_patch(circle_patch)
            ax.text(
                obj.x,
                obj.y + obj.r + 8,
                obj.name,
                ha="center",
                fontsize=9
            )

        elif isinstance(obj, Rectangle):
            color = "red" if is_colliding else "yellow"

            rectangle_patch = Polygon(
                obj.corners(),
                closed=True,
                alpha=0.6,
                color=color
            )

            ax.add_patch(rectangle_patch)

            corners = obj.corners()
            center_x = sum(point[0] for point in corners) / 4
            center_y = sum(point[1] for point in corners) / 4

            ax.text(
                center_x,
                center_y,
                obj.name,
                ha="center",
                va="center",
                fontsize=9
            )

    ax.set_title("Collision Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal")

    ax.set_xlim(0, 830)
    ax.set_ylim(0, 780)

    plt.savefig("collission_checker.png", dpi=150)
    plt.show()


def main():
    filename = "objects.txt"

    if not os.path.exists(filename):
        print(f"Error: {filename} was not found.")
        return

    objects = read_objects(filename)

    collisions, colliding_objects = find_collisions(objects)

    draw_objects(objects, colliding_objects)

    print("Collisions:")
    for obj1, obj2 in collisions:
        print(f"{obj1} collides with {obj2}")

    print()
    print(f"Total collisions: {len(collisions)}")
    print("Image saved as collision_output.png")


if __name__ == "__main__":
    main()