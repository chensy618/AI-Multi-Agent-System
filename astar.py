import heapq
from abc import ABCMeta, abstractmethod
from collections import deque

def astar(problem, conflicts):
    problem.start_position
    problem.goal_position 
    problem

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
        self.agents_goal_positions = {}
        self.boxes_goal_position = {}

        #Now we need to pick the goal for each agents,
        #keep in mind that state.goals return a matrix where the numer is the destination
        #for example, [['','','0'] ,['','',''] ,['','','']] means that we want the 0 agent in [0][2], so state.agent_cols[0] == 2, and state.agent_rows[0] == 0

        for row in range(len(initial_state.goals)):
            for col in range(len(initial_state.goals[row])):
                goal = initial_state.goals[row][col]
                if '0' <= goal <= '9':
                    agent_idx = int(goal)  # Convert goal to integer agent index
                    self.agents_goal_positions[agent_idx] = (row, col)  # Assign position tuple directly
                if 'A' <= goal <= 'Z':
                    box_goal = goal

                    if box_goal not in self.boxes_goal_position:
                        self.boxes_goal_position[box_goal] = []
                    self.boxes_goal_position[box_goal].append((row, col))

        #After that we'll have a dictionary agents_goal_positions that stores a tuple of the desired position for each agents, eg: (in the same example as before) agents_goal_positions[0] will return (0,2) 
                    
        print("#Hey, i just calculated the goal positions of the agents, they're here:",self.agents_goal_positions, flush=True)
        
        if(self.boxes_goal_position):
            print("#Hey, i just calculated the goal positions of the boxes, they're here:",self.boxes_goal_position, flush=True)


        #print("#Also, the walls are here: ",initial_state.walls)
        #print("#Hey, maybe you want to notice that the boxer are like this: ", initial_state.boxes)
        #print("#Hey, maybe you want to notice that the goal is like this: ", initial_state.goals)


    def h(self, state: 'State') -> 'int':
        #return self.goal_count_heuristic(state)
        #return self.calculate_distance(state)
        return self.heuristic_prime_boxes(state)

    def goal_count_heuristic(self, state : 'State') -> 'int':
        '''
        Needed for 4.2
        A heuristic function h(n) that simply counts how many goal cells are not yet covered by an object of the right type (so far all goal cells are agent goals, so h(n) should simply count how many agents are not at their destination)
        '''

        missing_agents = 0

        for agent_idx in range(len(state.agent_rows)):
            #where is the agent
            agent_row = state.agent_rows[agent_idx]
            agent_col = state.agent_cols[agent_idx]

            # Skip calculation if agent position is missing (None) or the agent doesn't have a goal
            if agent_row is None or agent_col is None or agent_idx not in self.agents_goal_positions:
                continue

            goal_row, goal_col = self.agents_goal_positions[agent_idx]

            if agent_row != goal_row and agent_col != goal_col:
                missing_agents+=1
        
        #return the agents that aren't in place
        return missing_agents

    def box_goal_count_heuristic(self, state : 'State') -> 'int':
        '''
        Supposing that during the levels where you have to move the boxes you don't have to find a goal state of the agents

        this is the goal count heuristic for Ex 6.1
        '''

        # Initialize a variable to store the count of boxes that aren't in the right position
        misplaced_boxes = 0

        # Iterate over the keys of self.boxes_goal_position that would be uppercase letters 'A', 'B' etc etc
        for box_key in self.boxes_goal_position:
            # Get the goal positions for the boxes with the current letter
            goal_positions = self.boxes_goal_position[box_key]
            
            # Iterate over each goal position for the current box
            for goal_position in goal_positions:
                # Extract the row and column of the current goal position
                goal_row, goal_col = goal_position
                
                # Check if the box is in the correct position according to the goal position
                if state.boxes[goal_row][goal_col] != box_key:
                    # Increment the count if the box is not in the right place
                    misplaced_boxes += 1

        return misplaced_boxes


    def calculate_distance(self, state) -> 'int':
        """
            Function to call for Exercise 4.3

            Tis function only canlcolate the manatthan distances for each of the agents from its destination and sums it up
            lower the number that this returned, closer are the agents from their destination
        """

        distances = [] #this will store all of the distances, distances[0] = how far is the agent 0 from its destination and go on...
        
        # Iterate over each agent
        for agent_idx in range(len(state.agent_rows)):
            #where is the agent
            agent_row = state.agent_rows[agent_idx]
            agent_col = state.agent_cols[agent_idx]
            
            # Skip calculation if agent position is missing (None)
            if agent_row is None or agent_col is None or agent_idx not in self.agents_goal_positions:
                continue
            
            #Get the goal position of the agent 'agent_idx'
            goal_row, goal_col = self.agents_goal_positions[agent_idx]
            
            # Calculate Manhattan distance if goal position is found
            if goal_row is not None and goal_col is not None:
                distance = abs(agent_row - goal_row) + abs(agent_col - goal_col)
                distances.append(distance)
            else:
                distances.append(None)  # Handle the case where goal position is not found
        
        return sum(distance for distance in distances if distance is not None)

    def heuristic_prime_boxes(self, state: "State") -> int:

        box_positions = {}
        #Iterate over self.boxes and estract the position of the boxes, then put in a dictionary where the keys are the letter of the boxes and the value are the position of the boxes with that letter
        for i, row in enumerate(state.boxes):
            for j, box in enumerate(row):
                if 'A' <= box <= 'Z':
                    if box not in box_positions:
                        box_positions[box] = [] 
                    box_positions[box].append((i, j))

        total_distance = 0

        # Iterate over the keys of self.boxes_goal_position that would be uppercase letters 'A', 'B' etc etc
        for box_key in self.boxes_goal_position:
            # Get the goal positions for the boxes with the current letter
            goal_positions_for_box_key = self.boxes_goal_position[box_key]
            # Get the current positions for the boxes with the current letter
            current_positions_for_box_key = box_positions[box_key]

            closest_mapping_for_key_boxes = self.find_closest_tuples_manhattan(goal_positions_for_box_key,current_positions_for_box_key)

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