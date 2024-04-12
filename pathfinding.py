from math import sqrt
from collections import namedtuple
from domain.position import Position
from domain.st_position import STPosition
from domain.agent import Agent
from domain.box import Box
from domain.goal import Goal
from domain.wall import Wall
from domain.color import Color
import heapq

class SpaceTimeAstar:
    def __init__(self, grid, agents, boxes, goals, walls):
        self.grid = grid
        self.agents = agents
        self.boxes = boxes
        self.goals = goals
        self.walls = walls
        self.reservation_table = {}

    def initialize_grid(self, x_dim, y_dim, max_time):
        #Initialize the 3D space-time grid.
        #Preset max_time
        max_time = sqrt(x_dim*x_dim + y_dim*y_dim)
        max_time = int(max_time) + 1
        # Construct grid
        grid = [[[False for _ in range(max_time)] for _ in range(y_dim)] for _ in range(x_dim)]
        # convert the agents, boxes, and walls to the grid
        copy_agents = [Agent(agent.pos, agent.id, agent.color) for agent in self.agents]
        copy_boxes = [Box(box.pos, box.id, box.color) for box in self.boxes]
        copy_walls = [Wall(wall.pos.position) for wall in self.walls]
        for wall in copy_walls:
            x, y = wall.pos.x, wall.pos.y
            for t in range(max_time):
                grid[x][y][t] = True # Mark the wall as occupied
        for agent in copy_agents:
            self.grid[agent.pos.x][agent.pos.y] = agent # Mark the agent as occupied
        for box in copy_boxes:
            self.grid[box.pos.x][box.pos.y] = box # Mark the box as occupied
        return grid
    
    def is_available(self, x, y, t):
        #Check if a cell is available at a given time.
        return (x,y,t) not in self.reservation_table and not self.grid[x][y]
    
    def reserve_cell(self, x, y, t):
        #Reserve a cell at a given time.
        self.reservation_table[(x,y,t)] = True

    def heuristic(self, current, goal):
        #Calculate the heuristic value.
        return abs(current.x - goal.x) + abs(current.y - goal.y)
    
    def get_neighbors(self, x, y, t):
        #Get the neighbors of a cell in the space-time grid.
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0)]  # 四个移动方向，包括暂停
        for dir_x, dir_y in directions:
            nei_x, nei_y = x + dir_x, y + dir_y
            nei_t = t + 1
            if 0 <= nei_x < len(self.grid) and 0 <= nei_y < len(self.grid[0]) and (nei_x, nei_y) not in self.walls:
                neighbors.append((nei_x, nei_y, nei_t))
        return neighbors
    
    def st_astar_search(self, agent, goal):
        #Perform space-time A* search.
        open_list = []
        start = STPosition(agent.pos.x, agent.pos.y, 0)
        goal = STPosition(goal.pos.x, goal.pos.y, 0)
        heapq.heappush(open_list, (self.heuristic(start, goal), start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while open_list:
            _, current = heapq.heappop(open_list)

            if (current.x, current.y) == (goal.x, goal.y):
                return self.reconstruct_path(came_from, start, current)

            for neighbor in self.get_neighbors(current.x, current.y, current.t):
                neighbor_position = STPosition(*neighbor)
                
                if self.is_available(neighbor_position.x, neighbor_position.y, neighbor_position.t):
                    new_cost = cost_so_far[current] + 1
                    if neighbor_position not in cost_so_far or new_cost < cost_so_far[neighbor_position]:
                        cost_so_far[neighbor_position] = new_cost
                        priority = new_cost + self.heuristic(neighbor_position, goal)
                        heapq.heappush(open_list, (priority, neighbor_position))
                        came_from[neighbor_position] = current
                        self.reserve_cell(neighbor_position.x, neighbor_position.y, neighbor_position.t)

    def reconstruct_path(self, came_from, start, current):
        path = []
        while current != start:
            path.append((current.x, current.y, current.t))
            current = came_from[current]
        path.append((start.x, start.y, start.t))
        path.reverse()
        return path

# Test the space-time A* search
# if __name__ == "__main__":
#     x_dim, y_dim, max_time = 10, 10, 20
#     obstacles = [(1, 1), (1, 2), (2, 1)]  # Define obstacles here
#     grid = [[None for _ in range(y_dim)] for _ in range(x_dim)]
#     agents = [Agent(Position(0,0), 0, Color.Blue)]
#     boxes = [Box(Position(1,1), 'A', Color.Blue)]
#     goals = [Goal(Position(9,9), 'A')]
#     walls = set(obstacles)

#     astar = SpaceTimeAstar(grid, agents, boxes, goals, walls)
#     astar.initialize_grid(x_dim, y_dim, max_time)
#     goal = goals[0]  # 假设我们目前只处理一个目标
#     agent = agents[0]
#     path = astar.st_astar_search(agent, goal)
#     print("Path:", path)
    
