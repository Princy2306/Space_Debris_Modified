# Space_Debris_Modified
Modified version of the Data Structures and Algorithms course project.


# Space Debris Collision Avoidance

A C++ project for finding safer routes for a spacecraft while avoiding space debris.

The project started as a basic A* pathfinding implementation. I modified it to also consider moving debris and collision risk while selecting a route.

## What it does

- Uses A* to find a route from start to goal
- Represents the environment as a graph
- Simulates debris with position and velocity
- Predicts where debris will be in the future
- Checks the distance between the route and debris
- Classifies collision risk as LOW, MEDIUM, HIGH, or CRITICAL
- Adds a penalty for risky routes
- Blocks routes with CRITICAL collision risk
- Compares normal A* with risk-aware A*

## How it works

The spacecraft moves through different points connected by edges.

For every possible route, the program checks whether moving debris could come close to the spacecraft.

The normal A* version mainly considers the distance/fuel cost.

The risk-aware version considers both fuel and collision risk:

    Total Cost = Fuel Cost + Risk Penalty

This allows the algorithm to choose a slightly longer route if it has much lower collision risk.

## Running the project

You need a C++17 compatible compiler.

Compile:

```bash
g++ -std=c++17 -Wall -Wextra -O2 src/collision_avoidance_system.cpp -o space_debris