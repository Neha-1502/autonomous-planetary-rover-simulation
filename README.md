# Autonomous Planetary Rover Simulation Using MATLAB

Models an autonomous rover crossing a 20x20 Mars-like grid with craters and rock
fields, sensing hazards, replanning with A*, and tracking mission metrics.
Base MATLAB only (R2018a or newer recommended). No toolboxes, Python or Simulink.

## Run
```matlab
cd('<path to this folder>')
main_rover            % two-panel simulation: press "Start Mission" / "Reset"
```
Optional name/value arguments, e.g. `main_rover('PauseTime', 0.03)`; see `help main_rover`.
`main_rover('AutoStart', true)` runs the mission immediately.

## Project structure
```text
main_rover.m             Two-panel UI: terrain + routes (left), telemetry (right), Start/Reset,
                         saves final_map.png, mission_dashboard.png, mission_metrics.csv
environment.m            20x20 terrain, start [2,2], goal [19,19]
rover_movement.m         One-cell 4-direction kinematics with collision/boundary checks
obstacle_detection.m     Range-based proximity sensor, route-blocked detection
astar_path_planning.m    From-scratch A* (Manhattan heuristic, 4-connected)
mission_controller.m     Mission loop: plan, sense, halt, replan, move; battery limit;
                         optional per-step callback used by the UI
calculate_metrics.m      Distance, moves, obstacles, replans, battery, completion, time
rover_visualization.m    Standalone animation of a route or a recorded mission
results/                 Generated images and CSV
```

## Battery model (simplified, not a physical power model)
100 energy units; each valid one-cell move costs 1 unit; sensing and planning are free.
The rover stops when the remaining energy is less than one move's cost, so the battery
never goes negative. Both numbers are parameters (`batteryParams`).

## Tests (run each script from the project folder)
```matlab
test_environment
test_astar_path_planning
test_rover_movement
test_obstacle_detection
test_mission_controller
test_calculate_metrics
test_rover_animation
test_path_visualization
test_visualization
test_mission_animation
test_main_rover
```