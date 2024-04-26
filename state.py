import random

from domain.action import Action, ActionType
from domain.agent import Agent
from domain.box import Box
from domain.color import Color
from domain.goal import Goal
from domain.position import Position
from domain.wall import Wall
from domain.constraint import Constraint

class State:
    _RNG = random.Random(1)

    # agents: list of Agents
    # boxes: list of Boxes
    def __init__(self, agents, boxes, goals):
        self.agents = agents
        self.boxes = boxes
        self.goals = goals
        self.parent = None
        self.joint_action = None
        self.constraints = None
        self.g = 0
        self._hash = None

    def result(self, joint_action: list[Action]) -> 'State':
        '''
        Returns the state resulting from applying joint_action in this state.
        Precondition: Joint action must be applicable and non-conflicting in this state.
        '''
        copy_agents = [Agent(agent.pos, agent.id, agent.color) for agent in self.agents]
        copy_boxes = [Box(box.pos, box.id, box.color) for box in self.boxes]
        copy_goals = [Goal(goal.pos, goal.id) for goal in self.goals]
        for agent_index, action in enumerate(joint_action):
            print(f"joint_action---{joint_action}")
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
                copied_box = next(box for box in copy_boxes if box.pos == box_pos)

                # Update the box's position
                copied_box.pos += action.box_rel_pos
                copied_agent.pos += action.agent_rel_pos

        # Create a new state with the updated agents and boxes
        copy_state = State(copy_agents, copy_boxes,copy_goals)
        copy_state.parent = self
        copy_state.joint_action = joint_action[:]
        copy_state.g = self.g + 1
        return copy_state

    def is_goal_state(self) -> bool:
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
                if agent_positions.get(goal.pos) != int(goal.id):
                    return False
            else:
                # If the goal ID is not recognized as a box or agent, return False
                print('There is something wrong with the goal id set up, please')
                return False
        return True

    def get_expanded_states(self) -> 'list[State]':
        num_agents = len(self.agents)
        #print(f"---num_agents---{num_agents}")

        # Determine list of applicable action for each individual agent.
        applicable_actions = [[action for action in Action if self.is_applicable(agentIdx, action)] for agentIdx in range(num_agents)]
        print(f"---applicable_actions---{applicable_actions}")
        # Iterate over joint actions, check conflict and generate child states.
        joint_action = [None for _ in range(num_agents)]
        actions_permutation = [0 for _ in range(num_agents)]
        expanded_states = []
        while True:
            for agentIdx in range(num_agents):
                joint_action[agentIdx] = applicable_actions[agentIdx][actions_permutation[agentIdx]]
                # print(f'---agentIdx---{agentIdx}')
                # print(f"---joint_action---{joint_action}")
            # if not self.is_conflicting(joint_action):
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

    def is_applicable(self, agent: int, action: Action) -> bool:
        agent = self.agents[agent]
        #print(f"---agent---{agent}")
        agent_destination = agent.pos + action.agent_rel_pos
        #print(f"---agent_destination---{agent_destination}")
        if action.type is ActionType.NoOp:
            return True

        elif action.type is ActionType.Move:
            return self.is_free(agent_destination)

        elif action.type is ActionType.Push:
            # Check if there is a box at the agent's destination to push
            box_to_push = next((box for box in self.boxes if box.pos == agent_destination), None)
            if box_to_push and box_to_push.color == agent.color:
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
            box_to_pull = next((box for box in self.boxes if box.pos == box_position), None)
            if box_to_pull and box_to_pull.color == agent.color:
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
        #print(f"---walls---{self.walls}")
        # Check if the position is occupied by a wall
        # is_wall = any(wall.pos.position == position for wall in self.walls)
        # return not is_wall and self.box_at(position) is None and self.agent_at(position) is None
        wall_flag = self.wall_at(position)
        box_flag = self.box_at(position)
        agent_flag = self.agent_at(position)
        #print(f"---wall_flag---{wall_flag},---box_flag---{box_flag},---agent_flag---{agent_flag},---position---{position}")
        return self.wall_at(position) is None and self.box_at(position) is None and self.agent_at(position) is None
    
    def agent_at(self, position: Position) -> Agent:
        for agent in self.agents:
            #if agent.pos.x == position.x and agent.pos.y == position.y:
            if agent.pos == position:
                return agent
        return None

    def box_at(self, position: Position) -> Box:
        for box in self.boxes:
            #if box.pos.x == position.x and box.pos.y == position.y:
            if box.pos == position:
                return box
        return None

    def goal_at(self, position: Position) -> Goal:
        for goal in self.goals:
            #if goal.pos.x == position.x and goal.pos.y == position.y:
            if goal.pos.postion == position:
                return goal
        return None
    
    def wall_at(self, position: Position) -> Wall:
        for wall in self.walls:
            # print(f"---wall---{wall.pos.position}")
            # print(f"---position---{position}")
            if wall.pos.position == position:
                return wall
        return None
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
            _hash = _hash * prime + hash(tuple(box.pos for box in self.boxes))
            _hash = _hash * prime + hash(tuple(box.color for box in self.boxes))
            _hash = _hash * prime + hash(tuple((goal.pos, goal.id) for goal in self.goals))
            walls_hash = hash(tuple(wall.pos for wall in self.walls))
            _hash = _hash * prime + walls_hash
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
        if any(g1.pos != g2.pos or g1.id != g2.id for g1, g2 in zip(self.goals, other.goals)):
            return False
        return True

    def __repr__(self):
        max_row = max(max(agent.pos.x for agent in self.agents), max(box.pos.x for box in self.boxes), max(wall.pos.position.x for wall in State.walls))
        max_col = max(max(agent.pos.y for agent in self.agents), max(box.pos.y for box in self.boxes), max(wall.pos.position.y for wall in State.walls))
        lines = []
        for row in range(max_row + 1):
            line = []
            for col in range(max_col + 1):
                pos = Position(row, col)
                agent = self.agent_at(pos)
                box = self.box_at(pos)
                wall = next((wall for wall in self.walls if wall.pos == pos), None)  # Added line
                if box is not None:
                    line.append(box.getRealBoxId())
                elif wall is not None:
                    line.append('+')
                elif agent is not None:
                    line.append(str(agent.id))
                else:
                    line.append(' ')
            lines.append(''.join(line))
        return '\n'.join(lines)


class SpaceTimeState(State):
    def __init__(self, agents, boxes, goals, time, constraints):
        super().__init__(agents, boxes, goals)
        self.time = time  # Add a time component to the state
        self.constraints = constraints

    # Modify is_applicable function to include constraints judgement
    def action_applicable(self, agent: int, action: Action) -> bool:
        agent = self.agents[agent]
        #print(f"---agent---{agent}")
        agent_destination = agent.pos + action.agent_rel_pos
        print(f"---agent_destination---{agent_destination}")
        if action.type is ActionType.NoOp:
            return True

        elif action.type is ActionType.Move:
            # Check if the agent's destination is free and not constrained
            return self.is_free(agent_destination) and not self.is_constrained(agent, agent_destination, self.time + 1)

        elif action.type is ActionType.Push:
            # Check if there is a box at the agent's destination to push
            box_to_push = next((box for box in self.boxes if box.pos == agent_destination), None)
            if box_to_push and box_to_push.color == agent.color:
                # Calculate the box's destination position
                box_destination = agent_destination + action.box_rel_pos
                # Check if both the agent's and the box's destinations are free and not constrained
                return self.is_free(box_destination) and not self.is_constrained(agent, box_destination, self.time + 1)
            else:
                return False

        elif action.type is ActionType.Pull:
            # Calculate the box's current position that the agent wants to pull
            box_position = agent.pos - action.box_rel_pos
            # Check if there is a box to pull at the calculated position
            box_to_pull = next((box for box in self.boxes if box.pos == box_position), None)
            if box_to_pull and box_to_pull.color == agent.color:
                # Check if the agent's destination is free and not constrained
                return self.is_free(agent_destination) and not self.is_constrained(agent, agent_destination, self.time + 1)
            else:
                return False

        return False


    def result(self, joint_action: list[Action]) -> 'SpaceTimeState':
        # Add the time dimension to the new state
        new_state = super().result(joint_action)
        return SpaceTimeState(new_state.agents, new_state.boxes, new_state.goals, self.time + 1, self.constraints)

    def get_expanded_states(self) -> 'list[SpaceTimeState]':
        num_agents = len(self.agents)
        print(f"agnet---{self.agents}")
        #print(f"---num_agents---{num_agents}")

        # Determine list of applicable action for each individual agent.
        applicable_actions = [[action for action in Action if self.action_applicable(agentIdx, action)] for agentIdx in range(num_agents)]
        print(f"---applicable_actions---{applicable_actions}")
        # Iterate over joint actions, check conflict and generate child states.
        joint_action = [None for _ in range(num_agents)]
        actions_permutation = [0 for _ in range(num_agents)]
        expanded_states = []
        while True:
            for agentIdx in range(num_agents):
                joint_action[agentIdx] = applicable_actions[agentIdx][actions_permutation[agentIdx]]
                # print(f'---agentIdx---{agentIdx}')
                print(f"---joint_action---{joint_action}")
            # if not self.is_conflicting(joint_action):

            # Generate the resulting state from the joint action.
            child_state = self.result(joint_action)
            print(f"---child_state---{child_state.agents, child_state.boxes, child_state.goals, child_state.time, child_state.constraints}")
            # Increment the time for the child state.
            child_state.time = self.time + 1
            expanded_states.append(child_state)

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

    def compare_position(self,st_pos,pos):
        return st_pos.x == pos.x and st_pos.y == pos.y

    def is_constrained(self, agent_index, position, time):
        print(f"---self.constraints---{self.constraints}")
        for constraint in self.constraints:
            print(f"--constraint.position:{constraint.pos}, position:{position}")
            #if constraint.agentId == agent_index and constraint.pos == position and constraint.t == time:
            result_pos = self.compare_position(constraint.pos,position)
            if constraint.agentId == agent_index and result_pos and constraint.t == time:
                return True
        return False
        

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
