"""Shortest collision-free paths around axis-aligned rectangular obstacles.

Pure geometry -- no ROS, no numpy -- so it can be read and tested on its own.
``path_planner_node.py`` is just the ROS wrapper around this.

The method is a *visibility graph*. With rectangular obstacles, any shortest
collision-free path is a chain of straight segments that only ever turns at an
obstacle corner. So:

  1. take the start, the goal, and every obstacle corner as graph nodes,
  2. connect the pairs with an unobstructed line of sight,
  3. run A* over that graph.

The result is exactly optimal, with no grid resolution to trade off.

Obstacles are plain dicts with ``x_min``/``y_min``/``x_max``/``y_max`` keys,
and points are ``(x, y)`` tuples. Everything is in metres.
"""

import math

# Obstacle rectangles are shrunk by this much before intersection tests, so a
# segment that legitimately *touches* a corner it is routing around is not
# rejected for "crossing" one of the two edges meeting at that corner.
EDGE_EPSILON = 1e-3
# Slightly smaller margin for the inside-a-box test, for the same reason.
INTERIOR_EPSILON = 1e-4
# Below this a cross product counts as zero. Coordinates are metres, so this is
# far below any real geometry.
COLLINEAR_EPSILON = 1e-9


def expand_bbox(bbox, margin):
    """Inflate a bounding box by ``margin`` on every side.

    Also normalises the corners: the vision node derives them by projecting
    image-space corners into the robot frame, which can swap min and max.
    """
    x_min, x_max = sorted((float(bbox['x_min']), float(bbox['x_max'])))
    y_min, y_max = sorted((float(bbox['y_min']), float(bbox['y_max'])))
    return {
        'x_min': x_min - margin,
        'y_min': y_min - margin,
        'x_max': x_max + margin,
        'y_max': y_max + margin,
    }


def point_inside_bbox(point, bbox):
    """True if ``point`` is strictly inside ``bbox`` (corners do not count)."""
    return (
        bbox['x_min'] + INTERIOR_EPSILON <= point[0] <= bbox['x_max'] - INTERIOR_EPSILON
        and bbox['y_min'] + INTERIOR_EPSILON <= point[1] <= bbox['y_max'] - INTERIOR_EPSILON
    )


def segments_intersect(p1, p2, q1, q2):
    """True if segment p1-p2 crosses or touches segment q1-q2."""

    def orientation(a, b, c):
        value = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
        if abs(value) < COLLINEAR_EPSILON:
            return 0                     # collinear
        return 1 if value > 0 else 2     # clockwise / counter-clockwise

    def on_segment(a, b, c):
        """True if b lies inside the bounding box of segment a-c."""
        return (
            min(a[0], c[0]) <= b[0] <= max(a[0], c[0])
            and min(a[1], c[1]) <= b[1] <= max(a[1], c[1])
        )

    o1, o2 = orientation(p1, p2, q1), orientation(p1, p2, q2)
    o3, o4 = orientation(q1, q2, p1), orientation(q1, q2, p2)

    # General case: each segment straddles the other's line.
    if o1 != o2 and o3 != o4:
        return True

    # Collinear special cases: an endpoint lying on the other segment.
    return (
        (o1 == 0 and on_segment(p1, q1, p2))
        or (o2 == 0 and on_segment(p1, q2, p2))
        or (o3 == 0 and on_segment(q1, p1, q2))
        or (o4 == 0 and on_segment(q1, p2, q2))
    )


def segment_intersects_bbox(p1, p2, bbox):
    """True if segment p1-p2 passes through the interior of ``bbox``."""
    # Catches a segment spanning the box diagonally with neither endpoint inside.
    midpoint = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)
    if point_inside_bbox(midpoint, bbox):
        return True

    x_min = bbox['x_min'] + EDGE_EPSILON
    y_min = bbox['y_min'] + EDGE_EPSILON
    x_max = bbox['x_max'] - EDGE_EPSILON
    y_max = bbox['y_max'] - EDGE_EPSILON

    for point in (p1, p2):
        if x_min <= point[0] <= x_max and y_min <= point[1] <= y_max:
            return True

    corners = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
    edges = zip(corners, corners[1:] + corners[:1])
    return any(segments_intersect(p1, p2, e1, e2) for e1, e2 in edges)


def build_nodes(start, goal, obstacles):
    """Start, goal, and every obstacle corner not buried inside another obstacle."""
    nodes = [start, goal]
    for obs in obstacles:
        corners = [
            (obs['x_min'], obs['y_min']),
            (obs['x_max'], obs['y_min']),
            (obs['x_max'], obs['y_max']),
            (obs['x_min'], obs['y_max']),
        ]
        nodes.extend(
            corner for corner in corners
            if not any(
                point_inside_bbox(corner, other)
                for other in obstacles if other is not obs
            )
        )
    # Overlapping obstacles can contribute the same corner twice.
    return list(set(nodes))


def build_graph(nodes, obstacles):
    """Adjacency list connecting every pair of nodes that can see each other.

    NOTE: O(n^2) node pairs, each tested against every obstacle. Fine for the
    handful of objects this robot ever sees; a larger scene would want a
    rotational-sweep visibility graph instead.
    """
    graph = {node: [] for node in nodes}
    for i, n1 in enumerate(nodes):
        for n2 in nodes[i + 1:]:
            if any(segment_intersects_bbox(n1, n2, obs) for obs in obstacles):
                continue
            dist = math.dist(n1, n2)
            graph[n1].append((n2, dist))
            graph[n2].append((n1, dist))
    return graph


def astar(graph, start, goal):
    """A* over ``graph`` using straight-line distance as the heuristic.

    Returns the list of nodes from start to goal, or None if goal is unreachable.

    NOTE: the open set is a plain set scanned with min(), so each pop is O(n)
    where a heap would be O(log n). Kept for readability -- the graph is tiny.
    """
    open_set = {start}
    came_from = {}

    g_score = {node: math.inf for node in graph}
    g_score[start] = 0.0
    f_score = {node: math.inf for node in graph}
    f_score[start] = math.dist(start, goal)

    while open_set:
        current = min(open_set, key=lambda node: f_score[node])

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        open_set.remove(current)

        for neighbor, weight in graph[current]:
            tentative_g = g_score[current] + weight
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + math.dist(neighbor, goal)
                open_set.add(neighbor)
    return None


def plan_path(start, goal, obstacles):
    """Shortest path from ``start`` to ``goal`` avoiding ``obstacles``.

    Returns a list of points beginning at ``start`` and ending at ``goal``.
    If no free path exists the straight line is returned, so the caller can
    still act (and the operator can see the margin was too tight).
    """
    # Nothing in the way: the straight line is already optimal.
    if not any(segment_intersects_bbox(start, goal, obs) for obs in obstacles):
        return [start, goal]

    graph = build_graph(build_nodes(start, goal, obstacles), obstacles)
    return astar(graph, start, goal) or [start, goal]


def _self_check():
    def length(path):
        return sum(math.dist(a, b) for a, b in zip(path, path[1:]))

    # No obstacles -> straight line.
    assert plan_path((0.0, 0.0), (1.0, 0.0), []) == [(0.0, 0.0), (1.0, 0.0)]

    # A box squarely in the way -> detour that clears it and is longer than the
    # blocked straight line but not absurdly so.
    box = {'x_min': 0.4, 'y_min': -0.2, 'x_max': 0.6, 'y_max': 0.2}
    path = plan_path((0.0, 0.0), (1.0, 0.0), [box])
    assert len(path) > 2, path
    assert path[0] == (0.0, 0.0) and path[-1] == (1.0, 0.0)
    assert not any(
        segment_intersects_bbox(a, b, box) for a, b in zip(path, path[1:])
    ), 'planned path cuts through the obstacle'
    assert 1.0 < length(path) < 1.6, length(path)

    # A box beside the line changes nothing.
    aside = {'x_min': 0.4, 'y_min': 0.5, 'x_max': 0.6, 'y_max': 0.9}
    assert plan_path((0.0, 0.0), (1.0, 0.0), [aside]) == [(0.0, 0.0), (1.0, 0.0)]

    # Fully walled in -> straight-line fallback rather than a crash.
    wall = {'x_min': -1.0, 'y_min': -1.0, 'x_max': 2.0, 'y_max': 1.0}
    assert plan_path((0.0, 0.0), (1.0, 0.0), [wall]) == [(0.0, 0.0), (1.0, 0.0)]

    # expand_bbox normalises flipped corners and inflates by the margin.
    grown = expand_bbox({'x_min': 0.6, 'x_max': 0.4, 'y_min': 0.2, 'y_max': -0.2}, 0.05)
    expected = {'x_min': 0.35, 'y_min': -0.25, 'x_max': 0.65, 'y_max': 0.25}
    assert all(math.isclose(grown[k], v, abs_tol=1e-9) for k, v in expected.items()), grown

    print('visibility_graph self-check passed')


if __name__ == '__main__':
    _self_check()
