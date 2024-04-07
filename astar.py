import heapq
from abc import ABCMeta, abstractmethod
from collections import deque
from state import State
from domain.position import Position
from domain.action import Action, ActionType

globals().update(Action.__members__)

def astar(problem_state, conflicts):
    # return [
    #         [MoveE],
    #         [MoveE],
    #         [MoveS],
    #     ]
    initial_state = problem_state
    goal_position = problem_state.goals[0]
    print(f"---goal_position--- {goal_position.pos}")
    frontier = FrontierBestFirst(HeuristicAStar(initial_state))
    frontier.add(initial_state)
    
    explored = set()

    while True:
        if frontier.is_empty():
            print("No solution found")
            return None  # No solution found

        current_state = frontier.pop()
        print(f"---current_state--- {current_state.agents[0].pos}")

        if current_state.is_goal_state():
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
        #return self.goal_count_heuristic(state)
        return self.calculate_distance(state)
        #return self.heuristic_prime_boxes(state)

    def calculate_distance(self, state) -> 'int':
        """
            Tis function only calculate the manatthan distances for each of the agents from its destination and sums it up
            lower the number that this returned, closer are the agents from their destination
        """
        total_distance = 0
        agent_to_goal_distance = 0
        box_to_goal_distance = 0
        if self.goal_agents != []:
            for goal in self.goal_agents: # loop in the goal agent list (agent_id, goal_row, goal_col)
                agent_id = int(goal[0]) # get the agent_id
                goal_pos = goal[1] # get the position
                print(f'#####agent_id is {agent_id}#######')
                print(f'#####goal_pos is {goal_pos}#######')
                agent_pos = state.agents[agent_id].pos
                distance = abs(agent_pos.x - goal_pos.x) + abs(agent_pos.y - goal_pos.y)
                agent_to_goal_distance += distance
                print(f'------------agent_to_goal_distance is {agent_to_goal_distance}-------------------')
        elif self.goal_boxes != []:
            for goal in self.goal_boxes: # loop in the goal box list (box_id, goal_row, goal_col)
                box_id = goal[0] # get the box value
                print(f'#####box_id is {box_id}#######')
                closest_box_distance = float('inf')
                # print(f'------------closest_box_distance is {closest_box_distance}-------------------')
                goal_pos = goal[1] # get the position
                print(f'#####goal_pos is {goal_pos}#######')
                box = next((b for b in state.boxes if b.id == box_id), None)
                if box:
                    box_pos = box.pos
                    print(f'#####box_pos is {box_pos}#######')
                    distance = abs(box_pos.x - goal_pos.x) + abs(box_pos.y - goal_pos.y)
                    closest_box_distance = min(closest_box_distance, distance)
                    # print(f'------------closest_box_distance in for loop is {closest_box_distance}-------------------')
                    if closest_box_distance != float('inf'):
                        box_to_goal_distance += closest_box_distance
                else:
                    print(f'No box found with ID {box_id}')
                    box_to_goal_distance += 0
            print(f'------------box_to_goal_distance in for loop is {box_to_goal_distance}-------------------')
        total_distance = agent_to_goal_distance + box_to_goal_distance
        print(f'------------total_distance is {total_distance}-------------------')
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

class HeuristicWeightedAStar(Heuristic):
    def __init__(self, initial_state: 'State', w: 'int'):
        super().__init__(initial_state)
        self.w = w

    def f(self, state: 'State') -> 'int':
        return state.g + self.w * self.h(state)

    def __repr__(self):
        return 'WA*({}) evaluation'.format(self.w)
