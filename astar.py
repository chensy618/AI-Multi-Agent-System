import heapq
from abc import ABCMeta, abstractmethod
from collections import deque
from state import State

def astar(problem_state, conflicts):
    #problem_state = State()
    initial_state = problem_state
    goal_position = problem_state.goals
    frontier = FrontierBestFirst(HeuristicAStar(initial_state))
    frontier.add(initial_state)

    explored = set()

    while True:
        if frontier.is_empty():
            return None  # No solution found

        current_state : State = frontier.pop()

        if current_state == goal_position:
            return current_state.extract_plan()  # Return the plan to reach the goal state

        explored.add(current_state)

        for state in current_state.get_expanded_states():
            if state not in explored and not frontier.contains(state):
                frontier.add(state)


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

class FrontierBestFirst(Frontier):
    def __init__(self, heuristic: 'Heuristic'):
        super().__init__()
        self.heuristic = heuristic
        self.priority_queue = PriorityQueue()
    
    def add(self, state: 'State'):
       self.priority_queue.push(state, self.heuristic.f(state))
    
    def pop(self) -> 'State':
        return self.priority_queue.pop()
    
    def is_empty(self) -> 'bool':
        return self.priority_queue.is_empty()
    
    def size(self) -> 'int':
        return len(self.priority_queue)
    
    def contains(self, state: 'State') -> 'bool':
        return  state in self.priority_queue
    
    def get_name(self):
        return 'best-first search using {}'.format(self.heuristic)

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
    
class Heuristic(metaclass=ABCMeta):
    def __init__(self, initial_state: 'State'):
        all_goals = initial_state.goals
        self.agent_goals = {}
        self.box_goals = {}

        for goal in all_goals:
            if 'A' <= goal.id <= 'Z':
                # Check if it's a box goal position with the matching ID
                self.box_goals.append(goal)
            elif '0' <= goal.id <= '9':
                # Check if it's an agent goal position with the matching ID
                self.agent_goals.append(goal)

    def h(self, state: 'State') -> 'int':
        #return self.goal_count_heuristic(state)
        #return self.calculate_distance(state)
        return self.heuristic_prime_boxes(state)

    def calculate_distance(self, state) -> 'int':
        """
            Tis function only calculate the manatthan distances for each of the agents from its destination and sums it up
            lower the number that this returned, closer are the agents from their destination
        """
        
        distances = []
        for agent in state.agents:
                   
            #Get the goal position of the agent
            for i, goal in enumerate(self.agent_goal):
                if goal.id == agent.id:
                    agent_index = i
                    break
            goal = self.agent_goals[agent_index]
            
            # Calculate Manhattan distance if goal position is found
            distance = abs(agent.pos.x - goal.pos.x) + abs(agent.pos.y - goal.pos.y)
            distances.append(distance)
        return distances

    def heuristic_prime_boxes(self, state: "State") -> int:

        total_distance = 0
        box_indexes = {}
        current_positions_of_box = []
        goal_positions_of_box = []
        box_ids = {}
        for box in state.boxes:
            box_ids.append(box.id)
        
        # Iterate over the keys of self.boxes_goal_position that would be uppercase letters 'A', 'B' etc etc
        for id in box_ids:    
            # Get all the goal postions of the same ID
            for box in self.box_goals:
                if box.id == id:
                    goal_positions_of_box.append(box.pos)
            
            # Get the current positions for the boxes with the current letter
            for i, current_box in enumerate(state.boxes):
                if box.id == current_box.id:
                    box_indexes.append(i)
                    current_positions_of_box.append(state.boxes[i].pos)    

            closest_mapping_for_key_boxes = self.find_closest_tuples_manhattan(box.pos,current_positions_of_box)

            for point1, point2 in closest_mapping_for_key_boxes.items():
                distance = abs(point2[0] - point1[0]) + abs(point2[1] - point1[1])
                total_distance += distance

        return total_distance
    
    def find_closest_tuples_manhattan(self, curr_positions_boxes, desired_positions_boxes):
        closest_mapping = {}
        array2_copy = desired_positions_boxes.copy()
        for point1 in curr_positions_boxes:
            closest_point = None
            min_distance = float('inf')
            for point2 in array2_copy:
                distance = abs(point2[0] - point1[0]) + abs(point2[1] - point1[1])
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point2
            closest_mapping[point1] = closest_point
            curr_positions_boxes.remove(point1)
            array2_copy.remove(closest_point)
        return closest_mapping

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

class HeuristicWeightedAStar(Heuristic):
    def __init__(self, initial_state: 'State', w: 'int'):
        super().__init__(initial_state)
        self.w = w
    
    def f(self, state: 'State') -> 'int':
        return state.g + self.w * self.h(state)
    
    def __repr__(self):
        return 'WA*({}) evaluation'.format(self.w)