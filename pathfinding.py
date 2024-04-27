from math import sqrt
from collections import deque, namedtuple
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
import memory
import time
import sys


start_time = time.perf_counter()

action_list = []

def print_search_status(explored, frontier):
    status_template = '#Expanded: {:8,}, #Frontier: {:8,}, #Generated: {:8,}, Time: {:3.3f} s\n[Alloc: {:4.2f} MB, MaxAlloc: {:4.2f} MB]'
    elapsed_time = time.perf_counter() - start_time
    print(status_template.format(len(explored), frontier.size(), len(explored) + frontier.size(), elapsed_time, memory.get_usage(), memory.max_usage), file=sys.stderr, flush=True)

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
                # print(f"---action--- {action}")
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
                    # print(f"---agent.pos--- {agent.pos}")
                    # print(f"---time--- {time}")
                    # print(f"---timeline--- {time_path}")
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
    
class Heuristic(metaclass=ABCMeta):
    def __init__(self, initial_state: 'State', goal_map):
        self.initial_state = initial_state
        self.goal_map = goal_map

    def h(self, state: 'State', problem) -> 'int':
        return self.calculate_heuristic_value(state, problem)


    def calculate_heuristic_value(self, state, problem) -> 'int':
        h = 0
        
        for agent in state.agents:
            if(problem[agent.id] == None):
                continue

            goal_box_uid = problem[agent.id]

            agent_goal_box = state.boxes[goal_box_uid]
            
            if(self.box_map[agent_goal_box.id] == None):
                continue

            # TODO: only works if the agent has one box otherwise problematic with the agent.id
            agent_to_box_dist
            if(self.box_map[agent_goal_box.id][agent_goal_box.pos.y][agent_goal_box.pos.x] != 0):
                agent_to_box_dist = self.manhatten_distance(agent.pos, agent_goal_box.pos)
            else:
                agent_to_box_dist = self.box_map[agent_goal_box.id][agent.pos.y][agent.pos.x]
            h += agent_to_box_dist
        
        for box in state.boxes:
            if(self.goal_map[box.id] == None):
                continue
            # TODO: only works if the agent has one goal otherwise problematic with the agent.id
            box_to_goal = self.goal_map[box.id][box.pos.y][box.pos.x]
            h += box_to_goal

        return h

    def manhatten_distance(self, pos1, pos2) -> 'int':
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)

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