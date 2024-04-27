import heapq
import random
from abc import ABCMeta, abstractmethod
from domain.action import Action, ActionType
from domain.position import Position
from domain.st_position import STPosition
from domain.agent import Agent
from domain.box import Box
from domain.goal import Goal
from domain.constraint import Constraint
from astar import construct_path_with_time



class SpaceTimeState:
    _RNG = random.Random(1)
    
    def __init__(self, agents, boxes, goals, time, constraints):
        self.agents = agents
        self.boxes = boxes
        self.goals = goals
        self.walls = None
        self.time = time
        self.constraints = constraints
        self.parent = None
        self.joint_action = None
        self.g = 0
        
    def st_result(self, agents_action : list[Action]) -> 'SpaceTimeState':
        '''
        Returns the state that results from executing the given action in this state.
        Preconditions: The action must be applicable in this state.
        '''
        copy_agents = [Agent(agent.pos, agent.id, agent.color) for agent in self.agents]
        copy_boxes = [Box(box.pos, box.id, box.color) for box in self.boxes]
        copy_goals = [Goal(goal.pos, goal.id) for goal in self.goals]
        for agent_index, action in enumerate(agents_action):
            copied_agent = copy_agents[agent_index]
            if action.type == Action.Move:
                copied_agent.pos = copied_agent.pos + action.agent_rel_pos # Update the agent's position
            elif action.type in [ActionType.Push, ActionType.Pull]: # Push or Pull
                box_pos = Position(-1, -1) # dummy value for initialization
                if action.type == ActionType.Push:
                    box_pos = copied_agent.pos + action.agent_rel_pos
                else:  # ActionType.Pull
                    box_pos = copied_agent.pos - action.box_rel_pos

                # Find the box object with the position near the agent
                copied_box = next(box for box in copy_boxes if box.pos == box_pos)

                # Update the box's position based on the action
                copied_box.pos += action.box_rel_pos
                # Update the agent's position based on the action
                copied_agent.pos += action.agent_rel_pos
        # Return the new state
        time_cost = self.time + 1
        constraints = self.constraints
        new_state = SpaceTimeState(copy_agents, copy_boxes, copy_goals, time_cost, constraints)
        new_state.parent = self
        new_state.g = self.g + 1
        return new_state
    
    def is_goal_state(self) -> 'bool':
        #Checks if this state is a goal state.
        
        # Create a mapping of box positions to their corresponding IDs
        box_positions = {box.pos: box.id for box in self.boxes}
        # Create a mapping of agent positions to their corresponding IDs
        agent_positions = {agent.pos: agent.id for agent in self.agents}

        # Check if all goals are satisfied by boxes and agents
        for goal in self.goals:
            if 'A' <= goal.id <= 'Z':
                # Check if there's a box at the goal position with the matching ID
                if box_positions.get(goal.pos) != goal.id:
                    return False
            elif '0' <= goal.id <= '9':
                # Check if there's an agent at the goal position with the matching ID
                if agent_positions.get(goal.pos) != int(goal.id):
                    return False
            else:
                # If the goal ID is not recognized as a box or agent, return False
                print('There is something wrong with the goal id set up, please')
                return False
        return True
    
    def get_expanded_states(self) -> 'list[SpaceTimeState]':
        '''
        Returns a list of all states reachable from this state.
        '''
        num_agents = len(self.agents)
        # Determine list of applicable action for each individual agent.
        applicable_actions = []
        for agentIdx in range(num_agents):
            agent_actions = []
            for action in Action:
                if self.is_applicable(agentIdx, action):
                    agent_actions.append(action)
            applicable_actions.append(agent_actions)
                      
        # Iterate over joint actions, check conflict and generate child states.
        joint_action = [None for _ in range(num_agents)]
        actions_permutation = [0 for _ in range(num_agents)]
        expanded_states = []
        while True:
            for agentIdx in range(num_agents):
                joint_action[agentIdx] = applicable_actions[agentIdx][actions_permutation[agentIdx]]
            expanded_states.append(self.st_result(joint_action))

            # Advance permutation.
            done = False
            for agent in range(num_agents):
                if actions_permutation[agent] < len(applicable_actions[agent]) - 1:
                    actions_permutation[agent] += 1
                    break
                else:
                    actions_permutation[agent] = 0
                    if agent == num_agents - 1:
                        done = True
            # Last permutation?
            if done:
                break
        SpaceTimeState._RNG.shuffle(expanded_states)
        return expanded_states


    def is_applicable(self, agent: 'Agent', action: 'Action') -> 'bool':
        '''
        Checks if the given action is applicable for the given agent in this state.
        '''
        # Get the new position of the agent after applying the action
        agent_new_pos = agent.pos + action.agent_rel_pos
        
        # check if the action is NoOp
        if action.type == ActionType.NoOp:
            return True
        
        elif action.type == ActionType.Move:
            # check if the agent's new postion is free and not constrained
            if self.is_wall(agent_new_pos) or self.is_box(agent_new_pos) or self.is_constraint_violated(agent, action):
                return False
            else:
                return True
            
        elif action.type == ActionType.Push:
            # Get the position of the box that the agent is trying to push
            box_to_push = next((box for box in self.boxes if box.pos == agent_new_pos), None)
            if box_to_push and box_to_push.color == agent.color:
                # Get the new position of the box after applying the action
                box_new_pos = box_to_push.pos + action.box_rel_pos
                # check if the box is at the new position and the agent's new position is free and not constrained
                if self.is_wall(box_new_pos) or self.is_box(box_new_pos) or self.is_constraint_violated(agent, action):
                    return False
                else:
                    return True
        elif action.type == ActionType.Pull:
            # Get the position of the box that the agent is trying to pull
            box_to_pull = next((box for box in self.boxes if box.pos == agent_new_pos), None)
            if box_to_pull and box_to_pull.color == agent.color:
                # Get the new position of the agent after applying the action
                agent_new_pos = agent.pos - action.box_rel_pos
                # check if the agent's new postion is free and not constrained
                if self.is_wall(agent_new_pos) or self.is_box(agent_new_pos) or self.is_constraint_violated(agent, action):
                    return False
                else:
                    return True
        else:
            print('Invalid action type')
            return False
        
    def is_agent(self, pos: 'Position') -> 'bool':
        #Checks if the given position is occupied by an agent.
        return pos in [agent.pos for agent in self.agents]
    
    def is_wall(self, pos: 'Position') -> 'bool':
        #Checks if the given position is occupied by a wall.
        return pos in [wall.pos for wall in self.walls]
    
    def is_box(self, pos: 'Position') -> 'bool':
        #Checks if the given position is occupied by a box.
        return pos in [box.pos for box in self.boxes]
    
    def is_constraint_violated(self, agent: 'Agent', action: 'Action') -> 'bool':
        #Checks if the given action violates any constraints.
        agent_new_pos = agent.pos + action.agent_rel_pos
        # check is what type of constraint : agent-agent, box-box, agent-box
        for id, constraint in self.constraints.items():
            if isinstance(id, int):
                if agent.id == id and agent_new_pos == constraint.pos and self.time == constraint.t:
                    return True
                else:
                    return False
            elif isinstance(id, str):
                find_box = next((box for box in self.boxes if box.pos == agent_new_pos), None)
                if action.type == ActionType.Push:
                    box_new_pos = find_box.pos + action.box_rel_pos
                    if box_new_pos == constraint.pos and self.time == constraint.t:
                        return True
                    else:
                        return False
                elif action.type == ActionType.Pull:
                    box_new_pos = find_box.pos - action.box_rel_pos
                    if box_new_pos == constraint.pos and self.time == constraint.t:
                        return True
                    else:
                        return False
            else:
                return False
        
    def extract_plan(self) -> list[Action]:
            plan = [None for _ in range(self.g)]
            state = self
            while state.joint_action is not None:
                plan[state.g - 1] = state.joint_action
                state = state.parent
            return plan

    def __eq__(self, other):
        if isinstance(other, SpaceTimeState):
            return self.agents == other.agents and self.boxes == other.boxes and self.goals == other.goals and self.time == other.time
        return False
    
    def __hash__(self):
        return hash((tuple(self.agents), tuple(self.boxes), tuple(self.goals), self.time))
    
    def __repr__(self):
        return f"SpaceTimeState(agents={self.agents}, boxes={self.boxes}, goals={self.goals}, time={self.time})"        

    
def space_time_astar(problem, constraints):
    agent = problem.agents
    box = problem.boxes
    goal = problem.goals
    initial_time = 0
    initial_state = SpaceTimeState(agent, box, goal, initial_time, constraints)
    frontier = FrontierBestFirst(HeuristicAStar(initial_state))
    frontier.add(initial_state)
    explored = set()
    while not frontier.is_empty():
        current_state = frontier.pop()
        if current_state.is_goal_state():
            plan = current_state.extract_plan()
            time_path = construct_path_with_time(agent,box,plan)
            return plan, time_path  # Return the plan to reach the goal state
        explored.add(current_state)
        for state in current_state.get_expanded_states():
            if state not in explored and not frontier.contains(state):
                frontier.add(state)
    return None  # No solution found
    
class STFrontier(metaclass=ABCMeta):
    @abstractmethod
    def add(self, state: 'SpaceTimeState'): raise NotImplementedError

    @abstractmethod
    def pop(self) -> 'SpaceTimeState': raise NotImplementedError

    @abstractmethod
    def is_empty(self) -> 'bool': raise NotImplementedError

    @abstractmethod
    def size(self) -> 'int': raise NotImplementedError

    @abstractmethod
    def contains(self, state: 'SpaceTimeState') -> 'bool': raise NotImplementedError

    @abstractmethod
    def get_name(self): raise NotImplementedError
    
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
    
class FrontierBestFirst(STFrontier):
    def __init__(self, heuristic: 'Heuristic'):
        super().__init__()
        self.heuristic = heuristic
        self.priority_queue = PriorityQueue()

    def add(self, state: 'SpaceTimeState'):
        self.priority_queue.push(state, self.heuristic.f(state))

    def pop(self) -> 'SpaceTimeState':
        return self.priority_queue.pop()

    def is_empty(self) -> 'bool':
        return self.priority_queue.is_empty()

    def size(self) -> 'int':
        return len(self.priority_queue)

    def contains(self, state: 'SpaceTimeState') -> 'bool':
        return  state in self.priority_queue

    def get_name(self):
        return 'best-first search using {}'.format(self.heuristic)

class Heuristic(metaclass=ABCMeta):
    def __init__(self, initial_state: 'SpaceTimeState'):
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

    def h(self, state: 'SpaceTimeState') -> 'int':
        return self.calculate_distance(state)

    def calculate_distance(self, state : 'SpaceTimeState') -> 'int':
        total_distance = 0
        agent_to_goal_distance = 0
        box_to_goal_distance = 0
        if self.goal_agents != []:
            for goal in self.goal_agents: # loop in the goal agent list (agent_id, goal_row, goal_col)
                agent_id = int(goal[0]) # get the agent_id
                goal_pos = goal[1] # get the position
                agent_pos = state.agents[agent_id].pos
                distance = abs(agent_pos.x - goal_pos.x) + abs(agent_pos.y - goal_pos.y)
                agent_to_goal_distance += distance
        elif self.goal_boxes != []:
            for goal in self.goal_boxes: # loop in the goal box list (box_id, goal_row, goal_col)
                box_id = goal[0] # get the box value
                closest_box_distance = float('inf')
                goal_pos = goal[1] # get the position
                box = next((b for b in state.boxes if b.id == box_id), None)
                if box:
                    box_pos = box.pos
                    distance = abs(box_pos.x - goal_pos.x) + abs(box_pos.y - goal_pos.y)
                    closest_box_distance = min(closest_box_distance, distance)
                    if closest_box_distance != float('inf'):
                        box_to_goal_distance += closest_box_distance
                else:
                    box_to_goal_distance += 0
        total_distance = agent_to_goal_distance + box_to_goal_distance
        return total_distance

    @abstractmethod
    def f(self, state: 'SpaceTimeState') -> 'int': pass

    @abstractmethod
    def __repr__(self): raise NotImplementedError

class HeuristicAStar(Heuristic):
    def __init__(self, initial_state: 'SpaceTimeState'):
        super().__init__(initial_state)

    def f(self, state: 'SpaceTimeState') -> 'int':
        g = state.g
        h = self.h(state)
        return g + h

    def __repr__(self):
        return 'A* evaluation'

class HeuristicWeightedAStar(Heuristic):
    def __init__(self, initial_state: 'SpaceTimeState', w: 'int'):
        super().__init__(initial_state)
        self.w = w

    def f(self, state: 'SpaceTimeState') -> 'int':
        return state.g + self.w * self.h(state)

    def __repr__(self):
        return 'WA*({}) evaluation'.format(self.w)