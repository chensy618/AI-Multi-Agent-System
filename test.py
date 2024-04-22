from collections import deque

# Initialize a 2D grid
grid = [
    [0, -1, 0, 0, 0],
    [0, -1, 0, 0, 0],
    [0, 0, -1, 0, 0],
    [0, 0, 0, 0, 0]
]

# Define the goal position
goal_position = (3, 3)

# Define movements (up, down, left, right)
movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]

# Function to calculate exact step count heuristic using BFS
def calculate_exact_step_count(grid, goal):
    queue = deque([(goal, 0)])  # (position, steps)
    visited = set()

    while queue:
        (x, y), steps = queue.popleft()

        if (x, y) in visited or grid[x][y] == -1:
            continue

        visited.add((x, y))
        grid[x][y] = steps

        for dx, dy in movements:
            new_x, new_y = x + dx, y + dy

            if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]):
                queue.append(((new_x, new_y), steps + 1))

# Calculate exact step count heuristic for each cell
calculate_exact_step_count(grid, goal_position)

# Function to print the grid
def print_grid(grid):
    for row in grid:
        print(row)

# Print the grid with heuristic values
print("Grid with Exact Step Count Heuristic:")
print_grid(grid)
print(grid[2][2])