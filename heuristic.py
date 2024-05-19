from abc import ABCMeta, abstractmethod
import heapq

from domain.task import SubTask
from helper.distance_calc import DistanceCalc
from state import State, SpaceTimeState



class Heuristic(metaclass=ABCMeta):
    def __init__(self, initial_state: 'State'):
        self.initial_state = initial_state

    def h(self, state: 'State', task) -> 'int':
        return self.calculate_heuristic_value(state, task)


    def calculate_heuristic_value(self, state: State, task) -> 'int':
        distance = 0
        
        agent = state.agents[0]

        if(isinstance(task, SubTask)):
            distance += DistanceCalc.manhatten_distance(task.goal_pos, agent.pos)
        else:

            if(task.box_uid == None or task.goal_uid == None):
                raise RuntimeError(f"Box_uid is None or goal_uid is None, this should not be happening")

            # Check if the task is a box-to-goal task or an agent-to-goal task
            if(task.box_uid == -1):
                distance += DistanceCalc.calculate_agent_task(agent, task.goal_uid)
            else:
                agent_box = state.get_box_by_uid(task.box_uid)
                distance += DistanceCalc.calculate_box_task(agent_box, agent, task.goal_uid)

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
    
class HeuristicSpaceTimeAStar(Heuristic):
    def __init__(self, initial_state: 'SpaceTimeState'):
        super().__init__(initial_state)

    def calculate_heuristic_value(self, state, task) -> int:
        # initialize distance and time cost
        distance = 0
        time_cost = 0
        agent = state.agents[0]

        if task.box_uid is None:
            raise RuntimeError("Box_uid or goal_uid is None, this should not be happening")

        # Check if the task is a box-to-goal task or an agent-to-goal task
        if task.box_uid == -1:
            # Agent to goal task
            distance = DistanceCalc.calculate_agent_task(agent, task.goal_uid)
            time_cost = distance  # assume agent moves at 1 cell per time step
        else:
            # Box to goal task
            agent_box = state.boxes[task.box_uid]
            distance = DistanceCalc.calculate_box_task(agent_box, agent, task.goal_uid)
            time_cost = distance * 1.5  # assume box moves at 1.5 cell per time step

        # return the sum of distance and time cost
        return distance + int(time_cost)
    
    def f(self, state: 'SpaceTimeState', task) -> 'int':
        g = state.g
        h = self.h(state, task)
        return g + h

    def __repr__(self):
        return 'Space Time A* evaluation'
    
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

