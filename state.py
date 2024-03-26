import random

from domain.action import Action, ActionType

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
    
    def result(self, joint_action) -> 'State':
        '''
        Returns the state resulting from applying joint_action in this state.
        Precondition: Joint action must be applicable and non-conflicting in this state.
        '''
        
        # Copy this state.
        copy_agent_rows = self.agent_rows[:]
        copy_agent_cols = self.agent_cols[:]
        copy_boxes = [row[:] for row in self.boxes]
        
        # Apply each action.
        for agent, action in enumerate(joint_action):
            if action.type is ActionType.NoOp:
                pass
            
            elif action.type is ActionType.Move:
                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta
            
        copy_state = State(copy_agent_rows, copy_agent_cols, copy_boxes)
        
        copy_state.parent = self
        copy_state.joint_action = joint_action[:]
        copy_state.g = self.g + 1
        
        return copy_state
    
    def is_goal_state(self) -> 'bool':
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                goal = State.goals[row][col]
                
                if 'A' <= goal <= 'Z' and self.boxes[row][col] != goal:
                    return False
                elif '0' <= goal <= '9' and not (self.agent_rows[ord(goal) - ord('0')] == row and self.agent_cols[ord(goal) - ord('0')] == col):
                    return False
        return True
    
    def get_expanded_states(self) -> '[State, ...]':
        num_agents = len(self.agent_rows)
        
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
        agent_row = self.agent_rows[agent]
        agent_col = self.agent_cols[agent]
        agent_color = State.agent_colors[agent]
        
        if action.type is ActionType.NoOp:
            return True
            
        elif action.type is ActionType.Move:
            destination_row = agent_row + action.agent_row_delta
            destination_col = agent_col + action.agent_col_delta
            return self.is_free(destination_row, destination_col)
                
    def is_conflicting(self, joint_action: '[Action, ...]') -> 'bool':
        num_agents = len(self.agent_rows)
        
        destination_rows = [None for _ in range(num_agents)] # row of new cell to become occupied by action
        destination_cols = [None for _ in range(num_agents)] # column of new cell to become occupied by action
        box_rows = [None for _ in range(num_agents)] # current row of box moved by action
        box_cols = [None for _ in range(num_agents)] # current column of box moved by action
        
        # Collect cells to be occupied and boxes to be moved.
        for agent in range(num_agents):
            action = joint_action[agent]
            agent_row = self.agent_rows[agent]
            agent_col = self.agent_cols[agent]
            
            if action.type is ActionType.NoOp:
                pass
            
            elif action.type is ActionType.Move:
                destination_rows[agent] = agent_row + action.agent_row_delta
                destination_cols[agent] = agent_col + action.agent_col_delta
                box_rows[agent] = agent_row # Distinct dummy value.
                box_cols[agent] = agent_col # Distinct dummy value.
                    
        for a1 in range(num_agents):
            if joint_action[a1] is Action.NoOp:
                continue
            
            for a2 in range(a1 + 1, num_agents):
                if joint_action[a2] is Action.NoOp:
                    continue
                
                # Moving into same cell?
                if destination_rows[a1] == destination_rows[a2] and destination_cols[a1] == destination_cols[a2]:
                    return True
                        
        return False
    
    def is_free(self, row: 'int', col: 'int') -> 'bool':
        return not State.walls[row][col] and self.boxes[row][col] == '' and self.agent_at(row, col) is None
    
    def agent_at(self, row: 'int', col: 'int') -> 'char':
        for agent in range(len(self.agent_rows)):
            if self.agent_rows[agent] == row and self.agent_cols[agent] == col:
                return chr(agent + ord('0'))
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
            _hash = _hash * prime + hash(tuple(self.agent_rows))
            _hash = _hash * prime + hash(tuple(self.agent_cols))
            _hash = _hash * prime + hash(tuple(State.agent_colors))
            _hash = _hash * prime + hash(tuple(tuple(row) for row in self.boxes))
            _hash = _hash * prime + hash(tuple(State.box_colors))
            _hash = _hash * prime + hash(tuple(tuple(row) for row in State.goals))
            _hash = _hash * prime + hash(tuple(tuple(row) for row in State.walls))
            self._hash = _hash
        return self._hash
    
    def __eq__(self, other):
        if self is other: return True
        if not isinstance(other, State): return False
        if self.agent_rows != other.agent_rows: return False
        if self.agent_cols != other.agent_cols: return False
        if State.agent_colors != other.agent_colors: return False
        if State.walls != other.walls: return False
        if self.boxes != other.boxes: return False
        if State.box_colors != other.box_colors: return False
        if State.goals != other.goals: return False
        return True
    
    def __repr__(self):
        lines = []
        for row in range(len(self.boxes)):
            line = []
            for col in range(len(self.boxes[row])):
                if self.boxes[row][col] != '': line.append(self.boxes[row][col])
                elif State.walls[row][col] is not None: line.append('+')
                elif self.agent_at(row, col) is not None: line.append(self.agent_at(row, col))
                else: line.append(' ')
            lines.append(''.join(line))
        return '\n'.join(lines)
