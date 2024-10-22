from collections import deque
import copy
import random
import sys
import time
from typing import Dict

from domain.action import Action, ActionType
from domain.agent import Agent
from domain.box import Box
from domain.color import Color
from domain.constraint import Constraint
from domain.goal import Goal
from domain.position import Position
from domain.task import Task
from domain.wall import Wall

class State:
    _RNG = random.Random(1)

    goal_map = {}
    box_goal_map = {}
    goals = []

    # agents: list of Agents
    # boxes: list of Boxes
    def __init__(self, agents: list[Agent], boxes: Dict[str, Box], walls):
        self.agents = agents
        self.boxes = boxes
        self.walls = walls
        self.parent = None
        self.joint_action = None
        self.g = 0
        self._hash = None

    def from_agent_perspective(self, agent_id, round):

        relaxed_agent = self.get_agent_by_uid(agent_id)

        agent_colored_boxes = self.get_agent_boxes(relaxed_agent.color)
        
        agents_not_in_round = self.agents_without_round(round)
        
        boxes_not_in_round = self.boxes_without_round(round)

        relaxed_walls = [row[:] for row in self.walls]

        for agent in agents_not_in_round:
            relaxed_walls[agent.pos.y][agent.pos.x] = True

        for box in boxes_not_in_round:
            relaxed_walls[box.pos.y][box.pos.x] = True
            
        return State([relaxed_agent], agent_colored_boxes, relaxed_walls)


    def get_agent_by_uid(self, agentId) -> Agent:
        for agent in self.agents:
            if agentId == agent.value:
                return Agent(agent.pos, agent.value, agent.color)
        

    def get_agent_boxes(self, color) -> Dict[str, Box]:
        return {box.uid: Box(box.pos, box.value, box.uid, box.color) for box in self.boxes.values() if box.color == color}

    def get_box_by_uid(self, box_uid) -> Box:
        return self.boxes.get(box_uid, Box(None, None, None, None))

    def get_goal_by_uid(self, goal_uid) -> Goal:
        return next((goal for goal in State.goals if goal.uid == goal_uid), None)
    
    def result(self, joint_action: list[Action]) -> 'State':
        '''
        Returns the state resulting from applying joint_action in this state.
        Precondition: Joint action must be applicable and non-conflicting in this state.
        '''
        copy_agents = [Agent(agent.pos, agent.value, agent.color) for agent in self.agents]
        copy_agents = sorted(copy_agents, key=lambda agent: agent.value)
        copy_boxes = copy.deepcopy(self.boxes)
        for agent_index, action in enumerate(joint_action):
            copied_agent = copy_agents[agent_index]
            if action.type == ActionType.Move:
                # Update the agent's position
                copied_agent.pos += action.agent_rel_pos
            elif action.type in [ActionType.Push, ActionType.Pull]:
                box_pos = Position(-1, -1)
                if action.type == ActionType.Push:
                    box_pos = copied_agent.pos + action.agent_rel_pos
                else:  # ActionType.Pull
                    box_pos = copied_agent.pos - action.box_rel_pos

                # Find the box object with the matching position
                copied_box = next((box for box in copy_boxes.values() if box.pos == box_pos), None)

                if(copied_box is None):
                    # print("All tasks are completed", file=sys.stderr)
                    break
                # Update the box's position
                copied_box.pos += action.box_rel_pos
                copied_agent.pos += action.agent_rel_pos

        # Create a new state with the updated agents and boxes
        copy_state = State(copy_agents, copy_boxes, self.walls)
        copy_state.parent = self
        copy_state.joint_action = joint_action[0]
        copy_state.g = self.g + 1
        return copy_state

    def agents_without_round(self, round):
        return [agent for agent in self.agents if agent.value not in round.keys()]
    
    def boxes_without_round(self, round):
        box_uids = [task.box_uid for task in round.values()]
        return [box for box in self.boxes.values() if box.uid not in box_uids]

    @staticmethod
    def initialize_goal_map(walls, goal_pos: Position):
        queue = deque([(goal_pos.x, goal_pos.y, 0)])  # (position, dist)
        visited = set()

        # Define movements (up, down, right, left)
        movements = [(0, -1), (0, 1), (1, 0), (-1, 0)]
        max_col = len(walls[0])
        max_row = len(walls)

        distance_grid = [[None] * max_col for _ in range(max_row)]

        while queue:
            (x, y, dist) = queue.popleft()

            if (x, y) in visited or walls[y][x]:
                continue

            visited.add((x, y))
            distance_grid[y][x] = dist

            for dx, dy in movements:
                new_x, new_y = x + dx, y + dy

                if 0 <= new_x < max_col and 0 <= new_y < max_row and (new_x, new_y) not in visited:
                    queue.append((new_x, new_y, dist + 1))
                    
        return distance_grid

    def is_goal_state_for_subgoal(self, task: Task, agent: Agent) -> bool:
        if(task.goal_uid == None):
            return True
        if(task.box_uid == -1):
            if State.goal_map[task.goal_uid][agent.pos.y][agent.pos.x] == 0:
                return True
        else:
            box = self.get_box_by_uid(task.box_uid)
            if State.goal_map[task.goal_uid][box.pos.y][box.pos.x] == 0:
                return True
        return False

    def get_expanded_states(self, task: Task) -> 'list[State]':
        num_agents = len(self.agents)

        # Determine list of applicable action for each individual agent.
        applicable_actions = [[action for action in Action if self.is_applicable(agentIdx, action, task)] for agentIdx in range(num_agents)]
        # Iterate over joint actions, check conflict and generate child states.
        joint_action = [None for _ in range(num_agents)]
        actions_permutation = [0 for _ in range(num_agents)]
        expanded_states = []
        while True:
            for agentIdx in range(num_agents):
                joint_action[agentIdx] = applicable_actions[agentIdx][actions_permutation[agentIdx]]

            expanded_states.append(self.result(joint_action))

            # Advance permutation.
            done = False    
            for agent_id in range(num_agents):
                if actions_permutation[agent_id] < len(applicable_actions[agent_id]) - 1:
                    actions_permutation[agent_id] += 1
                    break
                else:
                    actions_permutation[agent_id] = 0
                    if agent_id == num_agents - 1:
                        done = True
            # Last permutation?
            if done:
                break
        State._RNG.shuffle(expanded_states)
        return expanded_states

    def is_applicable(self, agent_id: int, action: Action, task: Task) -> bool:
        agent = self.agents[agent_id]
        agent_destination = agent.pos + action.agent_rel_pos
        if action.type is ActionType.NoOp:
            return True

        elif action.type is ActionType.Move:
            return self.is_free(agent_destination)

        elif action.type is ActionType.Push:
            # Check if there is a box at the agent's destination to push
            box_to_push = next((box for box in self.boxes.values() if box.pos == agent_destination), None)
            if box_to_push and box_to_push.uid == task.box_uid and box_to_push.color == agent.color:
                # Calculate the box's destination position
                box_destination = agent_destination + action.box_rel_pos
                # Check if the box's destination is free
                return self.is_free(box_destination)
            else:
                return False

        elif action.type is ActionType.Pull:
            # Calculate the box's current position that the agent wants to pull
            box_position = agent.pos - action.box_rel_pos
            # Check if there is a box to pull at the calculated position
            box_to_pull = next((box for box in self.boxes.values() if box.pos == box_position), None)
            if box_to_pull and box_to_pull.uid == task.box_uid and box_to_pull.color == agent.color:
                # Check if the agent's destination is free to move into
                return self.is_free(agent_destination)
            else:
                return False

        return False

    # def is_conflicting(self, joint_action: 'list[Action]') -> 'bool':
    #     num_agents = len(self.agents)

    #     destination_positions = [None for _ in range(num_agents)] # Position of new cell to become occupied by action
    #     box_positions = [None for _ in range(num_agents)] # current Position of box moved by action

    #     # Collect cells to be occupied and boxes to be moved.
    #     for agent_idx, action in enumerate(joint_action):
    #         agent_pos = self.agents[agent_idx].pos
    #         box_pos = None
    #         if action.type is ActionType.NoOp:
    #             pass
    #         elif action.type is ActionType.Move:
    #             destination_positions[agent_idx] = agent_pos + action.agent_rel_pos
    #             box_positions[agent_idx] = Position(agent_pos.x, agent_pos.y) # Distinct dummy value.
    #         if action.type in [ActionType.Push, ActionType.Pull]:
    #             # Calculate box's current and destination positions for Push/Pull
    #             if action.type == ActionType.Push:
    #                 box_pos = agent_pos + action.agent_rel_pos
    #                 box_destination_pos = box_pos + action.box_rel_pos
    #             else:  # ActionType.Pull
    #                 box_pos = agent_pos - action.box_rel_pos
    #                 box_destination_pos = agent_pos

    #         # Update the box positions to be checked for conflicts
    #         box_positions[agent_idx] = box_pos
    #         destination_positions[agent_idx] = box_destination_pos if action.type == ActionType.Push else agent_pos + action.agent_rel_pos

    #     for a1 in range(num_agents):
    #         if joint_action[a1].type is ActionType.NoOp:
    #             continue

    #         for a2 in range(a1 + 1, num_agents):
    #             if joint_action[a2].type is ActionType.NoOp:
    #                 continue

    #             # Moving into same cell?
    #             if (destination_positions[a1] is not None and destination_positions[a2] is not None and
    #                 destination_positions[a1] == destination_positions[a2]):
    #                 return True

    #     return False

    def is_free(self, position) -> bool:
        return not self.walls[position.y][position.x] and not self.box_at(position) and not self.agent_at(position)
    
    def is_goal_achieved(self, goal: Goal) -> bool:
        for box in self.boxes.values():
            if box.pos == goal.pos and box.value == goal.value:
                return True
        for agent in self.agents:
            if goal.value.isdigit():
                if agent.pos == goal.pos and agent.value == int(goal.value):
                    return True
        return False
    
    def goal_achieved_by_box(self, goal: Goal) -> Box:
        for box in self.boxes.values():
            if box.pos == goal.pos and box.value == goal.value:
                return box

    def agent_at(self, position: Position) -> Agent:
        for agent in self.agents:
            if agent.pos == position:
                return agent

    def box_at(self, position: Position) -> Box:
        for box in self.boxes.values():
            if box.pos == position:
                return box

    def extract_plan(self) -> list[Action]:
        plan = [None for _ in range(self.g)]
        state = self
        while state.joint_action is not None:
            plan[state.g - 1] = state.joint_action
            state = state.parent
        return plan


    def __hash__(self):
        if self._hash is None:
            prime = 31
            _hash = 1
            _hash = _hash * prime + hash(tuple(agent.pos for agent in self.agents))
            _hash = _hash * prime + hash(tuple(agent.color for agent in self.agents))
            _hash = _hash * prime + hash(tuple(box.pos for box in self.boxes.values()))
            _hash = _hash * prime + hash(tuple(box.color for box in self.boxes.values()))
            _hash = _hash * prime + hash(tuple((goal.pos, goal.value, goal.uid) for goal in State.goals))
            flattened_walls = tuple(tuple(row) for row in self.walls)
            _hash = _hash * prime + hash(flattened_walls)
            self._hash = _hash
        return self._hash

    def __eq__(self, other):
        if self is other:
            return True
        if not isinstance(other, State):
            return False
        if any(a1.pos != a2.pos or a1.color != a2.color for a1, a2 in zip(self.agents, other.agents)):
            return False
        if any(b1.pos != b2.pos or b1.color != b2.color for b1, b2 in zip(self.boxes.values(), other.boxes.values())):
            return False
        return True

    def __repr__(self):
        lines = []
        max_row = len(self.walls)
        max_col = len(self.walls[0])
        for row in range(max_row + 1):
            line = []
            for col in range(max_col + 1):
                pos = Position(col, row)
                agent = self.agent_at(pos)
                box = self.box_at(pos)
                wall = self.walls[row][col] if self.walls and row < len(self.walls) and col < len(self.walls[row]) else False
                if box is not None:
                    line.append(box.getRealBoxId())
                elif wall is not None:
                    line.append('+')
                elif agent is not None:
                    line.append(str(agent.value))
                else:
                    line.append(' ')
            lines.append(''.join(line))
        return '\n'.join(lines)



class SpaceTimeState(State):
    def __init__(self, agents, boxes, walls, time, constraints, g):
        super().__init__(agents, boxes, walls)
        self.parent = None
        self.joint_action = None
        self.constraints = constraints
        self.time = time  # Add a time component to the state
        self.g = g
        self._hash = None

    def from_agent_perspective(self, agent_id, round) -> 'SpaceTimeState': 

        relaxed_agent = self.get_agent_by_uid(agent_id)

        agent_colored_boxes = self.get_agent_boxes(relaxed_agent.color)
        
        relaxed_walls = [row[:] for row in self.walls]

        relaxed_constraints = [Constraint(constraint.agentId, constraint.pos, constraint.t) for constraint in self.constraints]
        relaxed_time = self.time

        return SpaceTimeState([relaxed_agent], agent_colored_boxes, relaxed_walls, relaxed_time, relaxed_constraints, self.g)

    # Modify is_applicable function to include constraints judgement
    def is_applicable(self, agent: int, action: Action, task: Task) -> bool:
        agent = self.agents[agent]
        #print(f"---agent---{agent}", file=sys.stderr)
        agent_destination = agent.pos + action.agent_rel_pos
        # print(f"---agent_destination---{agent_destination}",file=sys.stderr)
        if action.type is ActionType.NoOp:
            return True

        elif action.type is ActionType.Move:
            return self.is_free(agent_destination) and not self.is_constrained(agent.value, agent_destination, self.time + 1)

        elif action.type is ActionType.Push:
            # Check if there is a box at the agent's destination to push
            box_to_push = next((box for box in self.boxes.values() if box.pos == agent_destination), None)
            if box_to_push and box_to_push.uid == task.box_uid and  box_to_push.color == agent.color:
                # Calculate the box's destination position
                box_destination = agent_destination + action.box_rel_pos
                # Check if both the agent's and the box's destinations are free and not constrained
                return (self.is_free(box_destination) and not self.is_constrained(box_to_push.value, box_destination, self.time + 1))
            else:
                return False

        elif action.type is ActionType.Pull:
            # Calculate the box's current position that the agent wants to pull
            box_position = agent.pos - action.box_rel_pos
            # Check if there is a box to pull at the calculated position
            box_to_pull = next((box for box in self.boxes.values() if box.pos == box_position), None)
            if box_to_pull and box_to_pull.uid == task.box_uid and box_to_pull.color == agent.color:
                # Check if the agent's destination is free and not constrained
                return self.is_free(agent_destination) and not self.is_constrained(agent.value, agent_destination, self.time + 1)
            else:
                return False

        return False


    def result(self, joint_action: list[Action]) -> 'SpaceTimeState':
        '''
        Returns the state resulting from applying joint_action in this state.
        Precondition: Joint action must be applicable and non-conflicting in this state.
        '''
        copy_agents = [Agent(agent.pos, agent.value, agent.color) for agent in self.agents]
        copy_agents = sorted(copy_agents, key=lambda agent: agent.value)
        copy_boxes = copy.deepcopy(self.boxes)
        for agent_index, action in enumerate(joint_action):
            copied_agent = copy_agents[agent_index]
            if action.type == ActionType.Move:
                # Update the agent's position
                copied_agent.pos += action.agent_rel_pos
            elif action.type in [ActionType.Push, ActionType.Pull]:
                box_pos = Position(-1, -1)
                if action.type == ActionType.Push:
                    box_pos = copied_agent.pos + action.agent_rel_pos
                else:  # ActionType.Pull
                    box_pos = copied_agent.pos - action.box_rel_pos

                # Find the box object with the matching position
                copied_box = next((box for box in copy_boxes.values() if box.pos == box_pos), None)

                if(copied_box is None):
                    # print("All tasks are completed", file=sys.stderr)
                    break
                
                # Update the box's position
                copied_box.pos += action.box_rel_pos
                copied_agent.pos += action.agent_rel_pos

        # Create a new state with the updated agents and boxes
        copy_state = SpaceTimeState(copy_agents, copy_boxes, self.walls, self.time + 1, self.constraints, self.g + 1)
        copy_state.parent = self
        copy_state.joint_action = joint_action[0]
        return copy_state


    def is_constrained(self, agent_index, position, time):
        return any(constraint.agentId == agent_index and
                   constraint.pos == position and
                   constraint.t == time
                   for constraint in self.constraints)


    def __eq__(self, other):
        return (self.agents == other.agents and
                self.boxes == other.boxes and
                self.goals == other.goals and
                self.time == other.time)


    def __hash__(self):
        if self._hash is None:
            self._hash = (hash(tuple(self.agents)) ^
                          hash(tuple(self.boxes)) ^
                          hash(tuple(self.goals)) ^
                          hash(self.time))
        return self._hash
