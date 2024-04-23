from math import sqrt
from collections import namedtuple
from domain.position import Position
from domain.st_position import STPosition
from domain.agent import Agent
from domain.box import Box
from domain.goal import Goal
from domain.wall import Wall
from domain.color import Color
from domain.action import Action
from state import State
from abc import ABCMeta, abstractmethod
import heapq

action_list = []

class SpaceTimeAstar:
    def __init__(self, grid, agents, boxes, goals, walls, width, height):
        self.grid = grid
        self.agents = agents
        self.boxes = boxes
        self.goals = goals
        self.walls = walls
        self.width = width
        self.height = height
        self.reservation_table = {}

    def construct_path_with_time(self, agents, plan):
        time = 0
        time_path = {agent.id : [] for agent in agents}

        for actions in plan:
            time += 1
            for agent, action in zip(agents, actions):
                print(f"---action--- {action}")
                if action == Action.NoOp:
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.MoveN:
                    agent.pos = Position(agent.pos.x - 1, agent.pos.y)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.MoveS:
                    agent.pos = Position(agent.pos.x + 1, agent.pos.y)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.MoveE:
                    agent.pos = Position(agent.pos.x, agent.pos.y + 1)
                    time_path[agent.id].append((agent.pos, time))
                    print(f"---agent.pos--- {agent.pos}")
                    print(f"---time--- {time}")
                    print(f"---timeline--- {time_path}")
                elif action == Action.MoveW:
                    agent.pos = Position(agent.pos.x, agent.pos.y - 1)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.PushN:
                    agent.pos = Position(agent.pos.x - 1, agent.pos.y)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.PushS:
                    agent.pos = Position(agent.pos.x + 1, agent.pos.y)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.PushE:
                    agent.pos = Position(agent.pos.x, agent.pos.y + 1)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.PushW:
                    agent.pos = Position(agent.pos.x, agent.pos.y - 1)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.PullN:
                    agent.pos = Position(agent.pos.x - 1, agent.pos.y)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.PullS:
                    agent.pos = Position(agent.pos.x + 1, agent.pos.y)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.PullE:
                    agent.pos = Position(agent.pos.x, agent.pos.y + 1)
                    time_path[agent.id].append((agent.pos, time))
                elif action == Action.PullW:
                    agent.pos = Position(agent.pos.x, agent.pos.y - 1)
                    time_path[agent.id].append((agent.pos, time))
                else:
                    pass
        self.reservation_table = time_path
        return time_path


    def get_reconstructed_path(self):
        return self.reservation_table

    def st_astar(self,initial_state):
        frontier = AStarFrontier(HeuristicAStar(initial_state))
        frontier.add(initial_state)
        
        explored = set()

        while True:
            if frontier.is_empty():
                print("No solution found")
                return None  # No solution found

            current_state = frontier.pop()

            if current_state.is_goal_state():
                plan = current_state.extract_plan()
                time_path = self.construct_path_with_time(self.agents, plan)
                #print(f"---time_path--- {time_path}")
                return plan,time_path  # Return the plan to reach the goal state

            explored.add(current_state)

            for state in current_state.get_expanded_states():
                if state not in explored and not frontier.contains(state):
                    frontier.add(state)

            if len(explored) % 1000 == 0:
                print_search_status(explored, frontier)

            if memory.get_usage() > memory.max_usage:
                print_search_status(explored, frontier)
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                return None

class Heuristic(metaclass=ABCMeta):
    def __init__(self, initial_state: 'State'):
        # Initiate the value for parameters
        # create list to store the goal agent parameters
        self.goal_name_agent = []
        self.goal_position_agent = []
        # create list to store the goal box parameters
        self.goal_name_box = []
        self.goal_position_box = []

        # save the goal positions and the agent names
        for goal in range(len(initial_state.goals)):
            goal_id = initial_state.goals[goal].id
            position = initial_state.goals[goal].pos
            if goal_id.isdigit():
                self.goal_name_agent.append(goal_id)
                self.goal_position_agent.append(position)
            elif goal_id.isupper():
                self.goal_name_box.append(goal_id)
                self.goal_position_box.append(position)

        self.goal_agents = list(zip(self.goal_name_agent, self.goal_position_agent))
        self.goal_boxes = list(zip(self.goal_name_box, self.goal_position_box))
        print(f"---goal_agents--- {self.goal_agents}")
        print(f"---goal_boxes--- {self.goal_boxes}")

    def h(self, state: 'State') -> 'int':
        return self.calculate_distance(state)

    def calculate_distance(self, state) -> 'int':
        total_distance = 0
        agent_to_goal_distance = 0
        box_to_goal_distance = 0
        if self.goal_agents != []:
            for goal in self.goal_agents: # loop in the goal agent list (agent_id, goal_row, goal_col)
                agent_id = int(goal[0]) # get the agent_id
                goal_pos = goal[1] # get the position
                # print(f'#####agent_id is {agent_id}#######')
                # print(f'#####goal_pos is {goal_pos}#######')
                agent_pos = state.agents[agent_id].pos
                distance = abs(agent_pos.x - goal_pos.x) + abs(agent_pos.y - goal_pos.y)
                agent_to_goal_distance += distance
                # print(f'------------agent_to_goal_distance is {agent_to_goal_distance}-------------------')
        elif self.goal_boxes != []:
            for goal in self.goal_boxes: # loop in the goal box list (box_id, goal_row, goal_col)
                box_id = goal[0] # get the box value
                # print(f'#####box_id is {box_id}#######')
                closest_box_distance = float('inf')
                # print(f'------------closest_box_distance is {closest_box_distance}-------------------')
                goal_pos = goal[1] # get the position
                # print(f'#####goal_pos is {goal_pos}#######')
                box = next((b for b in state.boxes if b.id == box_id), None)
                if box:
                    box_pos = box.pos
                    # print(f'#####box_pos is {box_pos}#######')
                    distance = abs(box_pos.x - goal_pos.x) + abs(box_pos.y - goal_pos.y)
                    closest_box_distance = min(closest_box_distance, distance)
                    # print(f'------------closest_box_distance in for loop is {closest_box_distance}-------------------')
                    if closest_box_distance != float('inf'):
                        box_to_goal_distance += closest_box_distance
                else:
                    print(f'No box found with ID {box_id}')
                    box_to_goal_distance += 0
            # print(f'------------box_to_goal_distance in for loop is {box_to_goal_distance}-------------------')
        total_distance = agent_to_goal_distance + box_to_goal_distance
        # print(f'------------total_distance is {total_distance}-------------------')
        return total_distance

    @abstractmethod
    def f(self, state: 'State') -> 'int': pass

    @abstractmethod
    def __repr__(self): raise NotImplementedError

class HeuristicAStar(Heuristic):
    def __init__(self, initial_state: 'State'):
        super().__init__(initial_state)

    def f(self, state: 'State') -> 'int':
        g = state.g
        h = self.h(state)
        return g + h

    def __repr__(self):
        return 'A* evaluation'

class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._index = 0
        self._set = set()  # New set to store items

    def push(self, item, priority):
        if item not in self._set:  # Check if item is already in the set
            heapq.heappush(self._queue, (priority, self._index, item))
            self._index += 1
            self._set.add(item)  # Add item to set

    def pop(self):
        priority, _, item = heapq.heappop(self._queue)
        self._set.remove(item)  # Remove item from set
        return item

    def is_empty(self):
        return len(self._queue) == 0

    def __len__(self):
        return len(self._queue)

    def __contains__(self, item):
        return item in self._set

class Frontier(metaclass=ABCMeta):
    @abstractmethod
    def add(self, state: 'State'): raise NotImplementedError

    @abstractmethod
    def pop(self) -> 'State': raise NotImplementedError

    @abstractmethod
    def is_empty(self) -> 'bool': raise NotImplementedError

    @abstractmethod
    def size(self) -> 'int': raise NotImplementedError

    @abstractmethod
    def contains(self, state: 'State') -> 'bool': raise NotImplementedError

    @abstractmethod
    def get_name(self): raise NotImplementedError

class AStarFrontier(Frontier):
    def __init__(self, heuristic: 'Heuristic'):
        super().__init__()
        self.heuristic = heuristic
        self.priority_queue = PriorityQueue()

    def add(self, state: 'State'):
        self.priority_queue.push(state, state.g + self.heuristic.f(state))

    def pop(self) -> 'State':
        return self.priority_queue.pop()

    def is_empty(self) -> 'bool':
        return self.priority_queue.is_empty()

    def size(self) -> 'int':
        return len(self.priority_queue)

    def contains(self, state: 'State') -> 'bool':
        return state in self.priority_queue

    def get_name(self):
        return 'A* using {}'.format(self.heuristic)

# class SpaceTimeAstar:
#     def __init__(self, grid, agents, boxes, goals, walls):
#         self.grid = grid
#         self.agents = agents
#         self.boxes = boxes
#         self.goals = goals
#         self.walls = walls
#         self.reservation_table = {}

#     def initialize_grid(self, x_dim, y_dim, max_time):
#         #Initialize the 3D space-time grid.
#         #Preset max_time
#         max_time = sqrt(x_dim*x_dim + y_dim*y_dim)
#         max_time = int(max_time) + 1
#         # Construct grid
#         grid = [[[False for _ in range(max_time)] for _ in range(y_dim)] for _ in range(x_dim)]
#         # convert the agents, boxes, and walls to the grid
#         copy_agents = [Agent(agent.pos, agent.id, agent.color) for agent in self.agents]
#         copy_boxes = [Box(box.pos, box.id, box.color) for box in self.boxes]
#         copy_walls = [Wall(wall.pos.position) for wall in self.walls]
#         for wall in copy_walls:
#             x, y = wall.pos.x, wall.pos.y
#             for t in range(max_time):
#                 grid[x][y][t] = True # Mark the wall as occupied
#         for agent in copy_agents:
#             self.grid[agent.pos.x][agent.pos.y] = agent # Mark the agent as occupied
#         for box in copy_boxes:
#             self.grid[box.pos.x][box.pos.y] = box # Mark the box as occupied
#         return grid
    
#     def is_available(self, x, y, t):
#         #Check if a cell is available at a given time.
#         return (x,y,t) not in self.reservation_table and not self.grid[x][y]
    
#     def reserve_cell(self, x, y, t):
#         #Reserve a cell at a given time.
#         self.reservation_table[(x,y,t)] = True

#     def heuristic(self, current, goal):
#         #Calculate the heuristic value.
#         return abs(current.x - goal.x) + abs(current.y - goal.y)
    
#     def get_neighbors(self, x, y, t):
#         #Get the neighbors of a cell in the space-time grid.
#         neighbors = []
#         directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0)]  # 四个移动方向，包括暂停
#         for dir_x, dir_y in directions:
#             nei_x, nei_y = x + dir_x, y + dir_y
#             nei_t = t + 1
#             if 0 <= nei_x < len(self.grid) and 0 <= nei_y < len(self.grid[0]) and (nei_x, nei_y) not in self.walls:
#                 neighbors.append((nei_x, nei_y, nei_t))
#         return neighbors
    
#     def st_astar_search(self, agent, goal):
#         #Perform space-time A* search.
#         open_list = []
#         start = STPosition(agent.pos.x, agent.pos.y, 0)
#         goal = STPosition(goal.pos.x, goal.pos.y, 0)
#         heapq.heappush(open_list, (self.heuristic(start, goal), start))
#         came_from = {start: None}
#         cost_so_far = {start: 0}

#         while open_list:
#             _, current = heapq.heappop(open_list)

#             if (current.x, current.y) == (goal.x, goal.y):
#                 return self.reconstruct_path(came_from, start, current)

#             for neighbor in self.get_neighbors(current.x, current.y, current.t):
#                 neighbor_position = STPosition(*neighbor)

#                 if self.is_available(neighbor_position.x, neighbor_position.y, neighbor_position.t):
#                     new_cost = cost_so_far[current] + 1
#                     if neighbor_position not in cost_so_far or new_cost < cost_so_far[neighbor_position]:
#                         cost_so_far[neighbor_position] = new_cost
#                         priority = new_cost + self.heuristic(neighbor_position, goal)
#                         heapq.heappush(open_list, (priority, neighbor_position))
#                         came_from[neighbor_position] = current
#                         self.reserve_cell(neighbor_position.x, neighbor_position.y, neighbor_position.t)

#     def reconstruct_path(self, came_from, start, current):
#         path = []
#         while current != start:
#             path.append((current.x, current.y, current.t))
#             current = came_from[current]
#         path.append((start.x, start.y, start.t))
#         path.reverse()
#         return path

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
