# 🚀 Mars Rover Path Planning - CSCI 561 (Spring 2024)

This project implements three search algorithms (BFS, UCS, and A*) to solve a Mars rover path planning problem. The task simulates a rover moving across a terrain on Mars, seeking the most efficient path to collect soil samples while considering constraints like uphill energy and momentum.

## 📋 Problem Description

The goal is to guide a Mars rover from a start location to a goal location while traversing a terrain of safe locations and safe path segments. Each path segment has an associated cost, and the rover must respect energy constraints, especially when moving uphill. This project includes:
- **Breadth-First Search (BFS)**: Considers each move as having equal cost, while respecting energy constraints.
- **Uniform-Cost Search (UCS)**: Calculates path costs in 2D (ignoring elevation) while still considering energy rules.
- **A* Search (A\*)**: Uses 3D distances and a heuristic to compute optimal paths.

## 🔍 Algorithms Implemented

- **BFS (Breadth-First Search)**: Explores all neighboring locations at the present depth before moving on to nodes at the next depth level.
- **UCS (Uniform-Cost Search)**: Explores the least-cost path to each location, ignoring elevation but considering distance and step cost.
- **A* Search**: A heuristic search algorithm that uses 3D distances to prioritize paths that are expected to be closer to the goal.

## 🗺️ Key Features

- **Energy and Momentum Constraints**: The rover can only traverse uphill paths if it has enough energy or momentum from a prior downhill move.
- **Search Algorithms**: Three search algorithms are implemented to find the shortest path while accounting for energy constraints.
- **Input and Output**: The input file specifies the terrain map, the start/goal locations, and the energy limit. The output file contains the optimal path or "FAIL" if no path exists.

## 🚀 How It Works

The project reads an `input.txt` file specifying:
1. The algorithm to use (BFS, UCS, or A*).
2. The rover's energy limit for uphill moves.
3. A list of safe locations and their 3D coordinates.
4. A list of safe path segments between locations.

The output is an `output.txt` file that contains the shortest path (if found) or "FAIL" if no valid path exists.
