# --- my_mission.py ---
from retro_cflib import CrazyflieMock as Crazyflie
import math


def print_state(cf):
    state = cf.commander.get_state()
    print(
        f"State: x={state['x']:.2f}, y={state['y']:.2f}, z={state['z']:.2f}, "
        f"score={state['score']:.0f}"
    )


def distance_to_target(current_state, target_x, target_y, target_z):
    """Calculate distance between current position and target."""
    dx = current_state["x"] - target_x
    dy = current_state["y"] - target_y
    dz = current_state["z"] - target_z
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def main():
    print("Starting mission...")
    cf = Crazyflie()

    # Get the map
    world = cf.commander.get_map()
    grid = world["grid"]
    print(f"Map size: {world['width']} x {world['height']}")
    print("Obstacle map:")
    for row in grid:
        print(row)

    # Get batteries
    batteries = cf.commander.get_batteries()
    if not batteries:
        print("No batteries found on the map.")
        return

    print(f"Found {len(batteries)} battery(ies):")
    for b in batteries:
        print(f"  Battery at ({b['x']:.1f}, {b['y']:.1f})")

    # Sort batteries by distance from the start position
    start_x, start_y = 2.0, 2.0
    batteries = sorted(
        batteries,
        key=lambda b: (b["x"] - start_x) ** 2 + (b["y"] - start_y) ** 2,
    )

    takeoff_z = 1.0
    land_z = 0.1
    tolerance = 0.8  # go_to blocks until arrival within tolerance

    print("Taking off...")
    cf.commander.go_to(x=start_x, y=start_y, z=takeoff_z, tolerance=tolerance)
    print_state(cf)

    # Track previous position for fallback
    prev_x, prev_y, prev_z = start_x, start_y, takeoff_z
    current_x, current_y = start_x, start_y
    
    # Track which batteries have been visited
    remaining_batteries = list(enumerate(batteries, start=1))

    while remaining_batteries:
        # Sort remaining batteries by distance from current position
        remaining_batteries.sort(
            key=lambda item: (item[1]["x"] - current_x) ** 2 + (item[1]["y"] - current_y) ** 2
        )
        
        idx, battery = remaining_batteries[0]
        bx = battery["x"]
        by = battery["y"]
        print(f"\nFlying to battery {idx}/{len(batteries)} at ({bx:.1f}, {by:.1f})...")
        cf.commander.go_to(x=bx, y=by, z=takeoff_z, tolerance=tolerance)
        state = cf.commander.get_state()
        
        # Check if we actually reached the battery
        distance = distance_to_target(state, bx, by, takeoff_z)
        
        if distance < tolerance:
            # Successfully reached battery
            print(
                f"✓ Collected battery {idx}: position=({state['x']:.2f}, {state['y']:.2f}), "
                f"score={state['score']:.0f}"
            )
            prev_x, prev_y, prev_z = state["x"], state["y"], state["z"]
            current_x, current_y = state["x"], state["y"]
            # Remove this battery from remaining list
            remaining_batteries.pop(0)
        else:
            # Timeout - could not reach battery
            print(f"✗ TIMEOUT reaching battery {idx} at ({bx:.1f}, {by:.1f})")
            print(f"  Current position: ({state['x']:.2f}, {state['y']:.2f}), distance to target: {distance:.2f}m")
            print(f"  Returning to previous position ({prev_x:.1f}, {prev_y:.1f})...")
            
            # Go back to previous position
            cf.commander.go_to(x=prev_x, y=prev_y, z=prev_z, tolerance=tolerance)
            state = cf.commander.get_state()
            print(f"  Back at ({state['x']:.2f}, {state['y']:.2f})")
            print(f"  Skipping battery {idx}, trying next...")
            # Remove this battery from remaining list
            remaining_batteries.pop(0)

    print("All batteries visited. Returning to start and landing...")
    cf.commander.go_to(x=start_x, y=start_y, z=land_z, tolerance=tolerance)
    print_state(cf)

    print("Mission end.")


if __name__ == '__main__':
    main()
