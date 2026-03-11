#!/usr/bin/env python3
"""
Reflect a PathPlanner (.path) file over the horizontal line y = 4.0005.

Usage:
    python reflect_path.py <input_file> [output_file]

If no output file is given, the result is written to <input>_reflected.path
"""

import json
import sys
import os

REFLECT_Y = 4.0005


def reflect_y(y: float) -> float:
    """Reflect a y-coordinate over y = REFLECT_Y."""
    return 2 * REFLECT_Y - y


def reflect_point(pt: dict | None) -> dict | None:
    """Reflect a point {x, y} over the mirror line."""
    if pt is None:
        return None
    return {"x": pt["x"], "y": reflect_y(pt["y"])}


def reflect_rotation(deg: float) -> float:
    """
    Reflect a heading (degrees, PathPlanner convention) over a horizontal axis.

    PathPlanner stores headings as standard mathematical angles (CCW from +x).
    Reflecting over a horizontal line negates the y-component of the unit vector,
    which is equivalent to: angle' = -angle  (mod 360, kept in (-180, 180]).
    """
    reflected = -deg
    # Normalise to (-180, 180]
    while reflected > 180:
        reflected -= 360
    while reflected <= -180:
        reflected += 360
    return reflected


def reflect_waypoints(waypoints: list) -> list:
    out = []
    for wp in waypoints:
        new_wp = dict(wp)
        new_wp["anchor"] = reflect_point(wp["anchor"])
        new_wp["prevControl"] = reflect_point(wp.get("prevControl"))
        new_wp["nextControl"] = reflect_point(wp.get("nextControl"))
        out.append(new_wp)
    return out


def reflect_rotation_targets(targets: list) -> list:
    out = []
    for t in targets:
        new_t = dict(t)
        if "rotation" in new_t:
            new_t["rotation"] = reflect_rotation(new_t["rotation"])
        out.append(new_t)
    return out


def reflect_constraint_zones(zones: list) -> list:
    """Constraint zones reference waypoint positions, not coordinates — no change needed."""
    return zones


def reflect_point_towards_zones(zones: list) -> list:
    """pointTowardsZones reference field positions."""
    out = []
    for z in zones:
        new_z = dict(z)
        if "fieldPosition" in new_z and new_z["fieldPosition"] is not None:
            new_z["fieldPosition"] = reflect_point(new_z["fieldPosition"])
        out.append(new_z)
    return out


def reflect_event_markers(markers: list) -> list:
    """Event markers are tied to waypoint positions along the path — no coordinate change."""
    return markers


def reflect_goal_end_state(state: dict) -> dict:
    new_state = dict(state)
    if "rotation" in new_state and new_state["rotation"] is not None:
        new_state["rotation"] = reflect_rotation(new_state["rotation"])
    return new_state


def reflect_ideal_starting_state(state: dict) -> dict:
    new_state = dict(state)
    if "rotation" in new_state and new_state["rotation"] is not None:
        new_state["rotation"] = reflect_rotation(new_state["rotation"])
    return new_state


def reflect_path(data: dict) -> dict:
    result = dict(data)

    result["waypoints"] = reflect_waypoints(data.get("waypoints", []))
    result["rotationTargets"] = reflect_rotation_targets(data.get("rotationTargets", []))
    result["constraintZones"] = reflect_constraint_zones(data.get("constraintZones", []))
    result["pointTowardsZones"] = reflect_point_towards_zones(data.get("pointTowardsZones", []))
    result["eventMarkers"] = reflect_event_markers(data.get("eventMarkers", []))

    if "goalEndState" in data:
        result["goalEndState"] = reflect_goal_end_state(data["goalEndState"])

    if "idealStartingState" in data:
        result["idealStartingState"] = reflect_ideal_starting_state(data["idealStartingState"])

    return result


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_path = sys.argv[1]

    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        base, ext = os.path.splitext(input_path)
        output_path = f"{base}_reflected{ext}"

    with open(input_path, "r") as f:
        data = json.load(f)

    reflected = reflect_path(data)

    with open(output_path, "w") as f:
        json.dump(reflected, f, indent=2)

    print(f"Reflected path written to: {output_path}")


if __name__ == "__main__":
    main()