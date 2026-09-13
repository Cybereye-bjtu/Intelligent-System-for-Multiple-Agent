import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(first, second):
    x1, y1, yaw1 = first
    x2, y2, yaw2 = second
    cosine = math.cos(yaw1)
    sine = math.sin(yaw1)
    return (
        x1 + cosine * x2 - sine * y2,
        y1 + sine * x2 + cosine * y2,
        wrap(yaw1 + yaw2),
    )


def distance(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1]), abs(
        wrap(first[2] - second[2])
    )


def apply(transform, x, y):
    tx, ty, yaw = transform
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return tx + cosine * x - sine * y, ty + sine * x + cosine * y
