import random

from domain.action import Action, ActionType
from domain.agent import Agent
from domain.box import Box
from domain.color import Color
from domain.goal import Goal
from domain.position import Position

class State:
    _RNG = random.Random(1)

    # agents: list of Agents
    # boxes: list of Boxes
    def __init__(self, agents, boxes):
        self.agents = agents
        self.boxes = boxes
        self.parent = None
        self.joint_action = None
        self.g = 0
        self._hash = None

    def result(self, joint_action: '[Action, ...]') -> 'State':
        '''
        Returns the state resulting from applying joint_action in this state.
        Precondition: Joint action must be applicable and non-conflicting in this state.
        '''
        copy_agents = [Agent(agent.pos, agent.id, agent.color) for agent in self.agents]
        copy_boxes = [Box(box.pos, box.id, box.color) for box in self.boxes]
        for agent_index, action in enumerate(joint_action):
            agent = copy_agents[agent_index]
            if action.type == ActionType.Move:
                # Update the agent's position
                agent.pos = Position(agent.pos.x + action.agent_row_delta, agent.pos.y + action.agent_col_delta)
            elif action.type in [ActionType.Push, ActionType.Pull]:
                # Identify the box to be moved
                if action.type == ActionType.Push:
                    box_pos = Position(agent.pos.x + action.agent_row_delta, agent.pos.y + action.agent_col_delta)
                else:  # ActionType.Pull
                    box_pos = Position(agent.pos.x - action.box_row_delta, agent.pos.y - action.box_col_delta)

                # Find the box object with the matching position
                box = next(box for box in copy_boxes if box.pos == box_pos)

                # Update the box's position
                box.pos = Position(box.pos.x + action.box_row_delta, box.pos.y + action.box_col_delta)

                # Update the agent's position
                agent.pos = Position(agent.pos.x + action.agent_row_delta, agent.pos.y + action.agent_col_delta)

        # Create a new state with the updated agents and boxes
        copy_state = State(copy_agents, copy_boxes)
        copy_state.parent = self
        copy_state.joint_action = joint_action[:]
        copy_state.g = self.g + 1
        return copy_state

    def is_goal_state(self) -> 'bool':
        '''
        Checks if this state is a goal state.
        '''
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
                if agent_positions.get(goal.pos) != goal.id:
                    return False
            else:
                # If the goal ID is not recognized as a box or agent, return False
                print('There is something wrong with the goal id set up, please')
                return False
        return True

    def get_expanded_states(self) -> '[State, ...]':
        num_agents = len(self.agents)

        # Determine list of applicable action for each individual agent.
        applicable_actions = [[action for action in Action if self.is_applicable(agent, action)] for agent in range(num_agents)]

        # Iterate over joint actions, check conflict and generate child states.
        joint_action = [None for _ in range(num_agents)]
        actions_permutation = [0 for _ in range(num_agents)]
        expanded_states = []
        while True:
            for agent in range(num_agents):
                joint_action[agent] = applicable_actions[agent][actions_permutation[agent]]

            if not self.is_conflicting(joint_action):
                expanded_states.append(self.result(joint_action))

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
        State._RNG.shuffle(expanded_states)
        return expanded_states

    def is_applicable(self, agent: 'int', action: 'Action') -> 'bool':
        agent = self.agents[agent]
        agent_destination = Position(agent.pos.x + action.agent_row_delta, agent.pos.y + action.agent_col_delta)

        if action.type is ActionType.NoOp:
            return True

        elif action.type is ActionType.Move:
            return self.is_free(agent_destination)

        elif action.type is ActionType.Push:
            # Check if there is a box at the agent's destination to push
            box_to_push = next((box for box in self.boxes if box.pos == agent_destination), None)
            if box_to_push and box_to_push.color == agent.color:
                # Calculate the box's destination position
                box_destination = Position(agent_destination.x + action.box_row_delta, agent_destination.y + action.box_col_delta)
                # Check if the box's destination is free
                return self.is_free(box_destination)
            else:
                return False

        elif action.type is ActionType.Pull:
            # Calculate the box's current position that the agent wants to pull
            box_position = Position(agent.pos.x - action.box_row_delta, agent.pos.y - action.box_col_delta)
            # Check if there is a box to pull at the calculated position
            box_to_pull = next((box for box in self.boxes if box.pos == box_position), None)
            if box_to_pull and box_to_pull.color == agent.color:
                # Check if the agent's destination is free to move into
                return self.is_free(agent_destination)
            else:
                return False

        return False

    def is_conflicting(self, joint_action: '[Action, ...]') -> 'bool':
        num_agents = len(self.agents)

        destination_positions = [None for _ in range(num_agents)] # Position of new cell to become occupied by action
        box_positions = [None for _ in range(num_agents)] # current Position of box moved by action

        # Collect cells to be occupied and boxes to be moved.
        for agent_idx, action in enumerate(joint_action):
            agent_pos = self.agents[agent_idx].pos
            box_pos = None
            if action.type is ActionType.NoOp:
                pass
            elif action.type is ActionType.Move:
                destination_positions[agent_idx] = Position(agent_pos.x + action.agent_row_delta, agent_pos.y + action.agent_col_delta)
                box_positions[agent_idx] = Position(agent_pos.x, agent_pos.y) # Distinct dummy value.
            if action.type in [ActionType.Push, ActionType.Pull]:
                # Calculate box's current and destination positions for Push/Pull
                if action.type == ActionType.Push:
                    box_pos = Position(agent_pos.x + action.agent_row_delta, agent_pos.y + action.agent_col_delta)
                    box_destination_pos = Position(box_pos.x + action.box_row_delta, box_pos.y + action.box_col_delta)
                else:  # ActionType.Pull
                    box_pos = Position(agent_pos.x - action.box_row_delta, agent_pos.y - action.box_col_delta)
                    box_destination_pos = agent_pos

            # Update the box positions to be checked for conflicts
            box_positions[agent_idx] = box_pos
            destination_positions[agent_idx] = box_destination_pos if action.type == ActionType.Push else Position(agent_pos.x + action.agent_row_delta, agent_pos.y + action.agent_col_delta)

        for a1 in range(num_agents):
            if joint_action[a1].type is ActionType.NoOp:
                continue

            for a2 in range(a1 + 1, num_agents):
                if joint_action[a2].type is ActionType.NoOp:
                    continue

                # Moving into same cell?
                if (destination_positions[a1] is not None and destination_positions[a2] is not None and
                    destination_positions[a1].x == destination_positions[a2].x and
                    destination_positions[a1].y == destination_positions[a2].y):
                    return True

        return False

    def is_free(self, position) -> 'bool':
        return not State.walls[position.x][position.y] and self.box_at(position) is None and self.agent_at(position) is None

    def agent_at(self, position: 'Position') -> 'Agent':
        for agent in self.agents:
            if agent.pos.x == position.x and agent.pos.y == position.y:
                return agent
        return None

    def box_at(self, position: 'Position') -> 'Box':
        for box in self.boxes:
            if box.pos.x == position.x and box.pos.y == position.y:
                return box
        return None

    def extract_plan(self) -> '[Action, ...]':
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
            _hash = _hash * prime + hash(tuple(box.pos for box in self.boxes))
            _hash = _hash * prime + hash(tuple(box.color for box in self.boxes))
            _hash = _hash * prime + hash(tuple((goal.pos, goal.id) for goal in State.goals))
            _hash = _hash * prime + hash(tuple(tuple(row) for row in State.walls))
            self._hash = _hash
        return self._hash

    def __eq__(self, other):
        if self is other:
            return True
        if not isinstance(other, State):
            return False
        if any(a1.pos != a2.pos or a1.color != a2.color for a1, a2 in zip(self.agents, other.agents)):
            return False
        if any(b1.pos != b2.pos or b1.color != b2.color for b1, b2 in zip(self.boxes, other.boxes)):
            return False
        if State.walls != other.walls:
            return False
        if any(g1.pos != g2.pos or g1.id != g2.id for g1, g2 in zip(State.goals, other.goals)):
            return False
        return True

    def __repr__(self):
            max_row = max(max(agent.pos.x for agent in self.agents), max(box.pos.x for box in self.boxes), len(State.walls))
            max_col = max(max(agent.pos.y for agent in self.agents), max(box.pos.y for box in self.boxes), max(len(row) for row in State.walls))
            lines = []
            for row in range(max_row + 1):
                line = []
                for col in range(max_col + 1):
                    pos = Position(row, col)
                    agent = self.agent_at(pos)
                    box = self.box_at(pos)
                    if box is not None:
                        line.append(box.getRealBoxId())
                    elif State.walls[row][col]:
                        line.append('+')
                    elif agent is not None:
                        line.append(str(agent.id))
                    else:
                        line.append(' ')
                lines.append(''.join(line))
            return '\n'.join(lines)
