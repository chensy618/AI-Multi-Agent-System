from abc import ABCMeta, abstractmethod
import heapq
import sys
import time
import sys

from helper.distance_calc import DistanceCalc
from state import State



class Heuristic(metaclass=ABCMeta):
    def __init__(self, initial_state: 'State'):
        self.initial_state = initial_state

    def h(self, state: 'State', task) -> 'int':
        return self.calculate_heuristic_value(state, task)


    def calculate_heuristic_value(self, state, task) -> 'int':
        distance = 0
        
        for agent in state.agents:
            
            if(task.box_uid == None or task.goal_uid == None):
                raise RuntimeError(f"Box_uid is None or goal_uid is None, this should not be happening")

            # Check if agent needs to go to its agent goal
            if(task.box_uid == -1):
                agent_to_goal_dist = State.goal_map[task.goal_uid][agent.pos.y][agent.pos.x]
                distance += agent_to_goal_dist
            # If not, then agent needs to go to box
            else:
                # print(f"task.box_uid: {task.box_uid}", file=sys.stderr)
                # print(f"state.boxes: {state.boxes}", file=sys.stderr)
                agent_box = state.boxes[task.box_uid]
                # print(f"task.box_uid: {task.box_uid}", file=sys.stderr)
                # print(f"state.boxes: {state.boxes}", file=sys.stderr)
                # print(f"---agent_box---{agent_box}", file=sys.stderr)
                
                if(State.box_goal_map[agent_box.uid] == None):
                    continue

                agent_to_box_dist = DistanceCalc.pos_to_box_distance(agent_box, agent.pos)
                distance += agent_to_box_dist
                # print(f"---agent_to_box_dist---{agent_to_box_dist}", file=sys.stderr)
        for box in state.boxes:
            if(State.goal_map[task.goal_uid] == None):
                continue
            box_to_goal_dist = State.goal_map[task.goal_uid][box.pos.y][box.pos.x]
            distance += box_to_goal_dist

        return distance

    @abstractmethod
    def f(self, state: 'State', task) -> 'int': pass

    @abstractmethod
    def __repr__(self): raise NotImplementedError

class HeuristicAStar(Heuristic):
    def __init__(self, initial_state: 'State'):
        super().__init__(initial_state)

    def f(self, state: 'State', task) -> 'int':
        g = state.g
        h = self.h(state, task)
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

    def add(self, state: 'State', task):
        self.priority_queue.push(state, state.g + self.heuristic.f(state, task))

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

