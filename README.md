# TurtleBot Graph Search Path Planning

Graph-search based path planning for a TurtleBot grid-world environment. The planner supports Breadth-First Search, Greedy Best-First Search, Uniform Cost Search, A* Search, and a custom A* heuristic.

The project evaluates each search method using path search time and node expansion count across different grid dimensions and obstacle settings.

## Features

- TurtleBot grid-world search environment
- General graph-search framework
- Breadth-First Search
- Greedy Best-First Search
- Uniform Cost Search
- A* Search
- Custom A* heuristic
- Manhattan heuristic for goal-distance estimation
- Runtime evaluation
- Node-expansion evaluation
- CSV-based result logging
- Plot generation for performance comparison

## Algorithms

| Algorithm | Priority Function | Uses Path Cost | Uses Heuristic |
|---|---|---:|---:|
| BFS | FIFO / depth order | No | No |
| GBFS | h(s) | No | Yes |
| UCS | g(s) | Yes | No |
| A* | g(s) + h(s) | Yes | Yes |
| Custom A* | g(s) + h_custom(s) | Yes | Yes |

## Repository Structure

```text
config/      Runtime and environment configuration
docs/        Project notes and algorithm explanations
helpers/     Helper models and shared utilities
launch/      ROS launch files
msg/         ROS message definitions
scripts/     Search, evaluation, and plotting scripts
srv/         ROS service definitions
tests/       Test scripts
worlds/      TurtleBot simulation worlds
results/     CSV files and generated plots
