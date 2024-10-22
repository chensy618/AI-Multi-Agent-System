import itertools
import sys
import time

from astar import astar
from cbs.node import Node
from cbs.agent_coop import ask_blocked_agent_help, meta_agent_block_communication
from cbs.utils import get_actual_agent_id, merge_plans
from domain.action import Action, ActionType
from domain.conflict import Conflict, MoveAwayConflict,FollowConflict, MetaAgentConflict
from domain.constraint import Constraint
from domain.position import Position
from queue import PriorityQueue

from st_astar import space_time_a_star
from state import State

# Create a counter that will serve as a tiebreaker
tiebreaker = itertools.count()

def conflict_based_search(current_state: State, round):
    # print(f"\n=============CBS-begin===============", file=sys.stderr)
    root = Node()
    root.constraints = set()
    initial_positions = initialize_initial_positions(current_state, round)
    for agent_uid, task in round.items():
        agent = current_state.get_agent_by_uid(agent_uid)
        relaxed_state = current_state.from_agent_perspective(agent.value, round)
        plan = astar(relaxed_state, round[agent.value])
        if plan == None:
            for goal in current_state.goals:
                if goal.uid == round[agent.value].goal_uid:
                    goals = [g for g in current_state.goals if g.group == goal.group and g.uid != goal.uid]
                    goal.group = -goal.group
                    for g in goals:
                        g.group = -g.group
        root.solution[agent.value] = plan

    root.cost = cost(root.solution)
    frontier = PriorityQueue()
    count_next = next(tiebreaker)
    frontier.put((count_next, root))

    # Store the initial solution to be used in the long corridor situation
    initial_solutions = root.solution.copy()
    # print(f'"=============Initial solutions: =============\n{initial_solutions}', file=sys.stderr)
    # Create a dictionary to store the number of conflicts for each agent pair
    conflict_counts = {}

    while not frontier.empty():
        tiebreaker_value, node = frontier.get()
        conflict = find_first_conflict(node.solution, initial_positions, conflict_counts)
        if conflict is None:
            executable_plan = merge_plans(current_state, node.solution, round)
            return executable_plan

        elif  isinstance(conflict, MoveAwayConflict):
            m = node.copy()
            new_node = solve_moveaway_conflict(node, conflict, current_state.walls)
            new_conflict = find_first_conflict(new_node.solution, initial_positions, conflict_counts)
            if new_conflict is None:
                executable_plan = merge_plans(current_state, new_node.solution, round)
                return executable_plan
            else:
                new_node.cost = cost(new_node.solution)
                if new_node.cost < sys.maxsize:
                    count_next = next(tiebreaker)
                    frontier.put((count_next, new_node))
                continue
        elif  isinstance(conflict, FollowConflict):
            m = node.copy()
            new_node = solve_follow_conflict(node, conflict)
            new_conflict = find_first_conflict(new_node.solution, initial_positions, conflict_counts)
            if new_conflict is None:
                executable_plan = merge_plans(current_state, new_node.solution, round)
                return executable_plan
            else:
                new_node.cost = cost(new_node.solution)
                if new_node.cost < sys.maxsize:
                    count_next = next(tiebreaker)
                    frontier.put((count_next, new_node))
                continue
        elif isinstance(conflict, MetaAgentConflict):
            m = node.copy()
            if round[conflict.ai].box_uid == -1 and round[conflict.aj].box_uid == -1:
                new_node = solve_meta_agent_conflict(node, conflict, initial_solutions)
                new_conflict = find_first_conflict(new_node.solution, initial_positions, conflict_counts)
                if new_conflict is None:
                    executable_plan = merge_plans(current_state, new_node.solution, round)
                    return executable_plan
                else:
                    new_node.cost = cost(new_node.solution)
                    if new_node.cost < sys.maxsize:
                        count_next = next(tiebreaker)
                        frontier.put((count_next, new_node))
                    continue
            else:
                # If meta agent cannot finish it's route, then it enters the communication mode to ask other agent's help
                new_node = meta_agent_block_communication(node, initial_solutions, initial_positions, conflict, current_state, round)
                new_conflict = find_first_conflict(new_node.solution, initial_positions, conflict_counts)
                if new_conflict is None:
                    executable_plan = merge_plans(current_state, new_node.solution, round)
                    return executable_plan
                else:
                    new_node.cost = cost(new_node.solution)
                    if new_node.cost < sys.maxsize:
                        count_next = next(tiebreaker)
                        frontier.put((count_next, new_node))
                continue
        else:
            ai_aj_pair = (conflict.ai, conflict.aj)
            conflict_counts[ai_aj_pair] = conflict_counts.get(ai_aj_pair, 0) + 1
            resolve_conflict(node, conflict, current_state, round, frontier)

    # print(f"CBS cannot resolve conflict", file=sys.stderr)
    # print(f"=============CBS-end===============\n", file=sys.stderr)
    return None

def resolve_conflict(node, conflict, current_state, round, frontier):
    for agent_uid, task in round.items():
        entity_id = None
        if(agent_uid in [conflict.ai, conflict.aj]):
            entity_id = agent_uid
        else:
            box = current_state.get_box_by_uid(task.box_uid)
            entity_id = box.value
        m = node.copy()
        # Define the constraints based on the conflict
        new_constraints = {
            Constraint(entity_id, Position(conflict.pos.x, conflict.pos.y), conflict.t),
            Constraint(entity_id, Position(conflict.pos.x, conflict.pos.y), conflict.t+1),
            Constraint(entity_id, Position(conflict.pos.x, conflict.pos.y), conflict.t+2)
        }
        # Add the new constraints if they don't already exist
        for constraint in new_constraints:
            if constraint not in m.constraints:
                m.constraints.add(constraint)
        relaxed_state = current_state.from_agent_perspective(agent_uid, round)
        st_solution = space_time_a_star(relaxed_state, m.constraints, round[agent_uid])
        m.solution[agent_uid] = st_solution
        m.cost = cost(m.solution)
        if m.cost < sys.maxsize:
            count_next = next(tiebreaker)
            frontier.put((count_next, m))
    return m

def initialize_initial_positions(current_state, round):
    initial_positions = {}

    for agent_uid, task in round.items():
        agent = current_state.get_agent_by_uid(agent_uid)
        box = current_state.get_box_by_uid(task.box_uid)
        goal = current_state.get_goal_by_uid(task.goal_uid)
        initial_positions[agent.value] = {
                'agent_position': agent.pos,
                'box_id': box.value if box else None,
                'box_position': box.pos if box else None,
                'goal_id': goal.value if goal else None,
                'goal_position': goal.pos if goal else None
            }
    return initial_positions

def cost(solution):
    """
    Calculate the total cost of a solution, which is the sum of the lengths of all paths.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: The total cost as an integer.
    """
    total_cost = 0
    for path in solution.values():
        if path is not None:
            total_cost += len(path)  # Add the length of this agent's path to the total cost
    return total_cost

def find_first_conflict(solution, initial_positions, conflict_counts):
    """
    Find the first conflict in the solution.

    :param solution: A dictionary mapping entity(agent/box) IDs to their respective paths (lists of actions).
    :return: A conflict object with details about the conflict, or None if no conflict is found.
    """
    if not solution:
        print('find_first_conflict: No solution found from astar.', file=sys.stderr)
        return None
    # Create a dictionary to track positions of each entity at each time step
    positions = {}
    avoid_pos_list = {}
    # Find the maximum path length to know how many time steps to check
    max_path_length = max(len(path) for path in solution.values())

    # sort agent by their path length, so that we always start from short path
    sorted_agents = sorted(solution.items(), key=lambda item: len(item[1]))

    for agent_id, path in sorted_agents:

        # agent-agent conflict
        if initial_positions[agent_id]['box_id'] is None:
            current_position = initial_positions[agent_id]['agent_position']
            # Add initial position to the positions dictionary as time step 0
            positions[(current_position, 0)] = {'id': agent_id}
            time_step = 1 # Start the first step at 1
            # Loop over the max path length to avoid situation that one agent reach the goal and stop moving, but still block the other agents
            while time_step < len(path)+1:
                # Get the action for the current time step
                action = path[time_step-1]
                # Calculate the resulting position of the agent after the action
                resulting_position = Position(
                    current_position.x + action.agent_rel_pos.x,
                    current_position.y + action.agent_rel_pos.y
                )
                if (resulting_position, time_step) in positions:
                    # Conflict detected, return information about the conflict
                    other_agent_id = positions[(resulting_position, time_step)]['id']
                    if conflict_counts.get((agent_id, other_agent_id), 0) < 3:
                        return Conflict(agent_id, other_agent_id, resulting_position, time_step)
                    else:
                        return MetaAgentConflict(agent_id, other_agent_id)
                ### Following conflict one way ###
                # Other agent is moving into the current agent's position at the timestep
                if (current_position, time_step) in positions:
                    other_agent_id = positions[(current_position, time_step)]['id']
                    # remove situation that the agent id is itself because of NoOp action
                    if other_agent_id != agent_id:
                        return FollowConflict(other_agent_id, time_step-1)
                ### Following conflict the other way ###
                # Current agent is moving into the other agent's position at the timestep
                if (resulting_position, time_step-1) in positions:
                    other_agent_id = positions[(resulting_position, time_step-1)]['id']
                    # remove situation that the agent id is itself because of NoOp action
                    if other_agent_id != agent_id:
                        return FollowConflict(agent_id, time_step-1)
                # update the position of the agent at specific time step
                positions[(resulting_position, time_step)] = {'id': agent_id}
                # Update the current position of the agent
                current_position = resulting_position
                time_step += 1

        # agent-box conflict
        # box-box conflict
        else:
            current_agent_position = initial_positions[agent_id]['agent_position']
            box_id = initial_positions[agent_id]['box_id']
            current_box_position = initial_positions[agent_id]['box_position']

            # Add initial positions to the positions dictionary as time step 0, and as resulting positions
            positions[(current_agent_position, 0)] = {'id': agent_id, 'tag': 'fixed'}
            positions[(current_box_position, 0)] = {'id': box_id, 'tag': 'fixed'}

            time_step = 1
            # Iterate for max_path_length steps
            for time_step in range(1, max_path_length + 1):
                # Agent is within its path
                if time_step < len(path)+1:
                    action = path[time_step - 1]
                    resulting_agent_position = Position(
                        current_agent_position.x + action.agent_rel_pos.x,
                        current_agent_position.y + action.agent_rel_pos.y
                    )
                    resulting_box_position = Position(
                        current_box_position.x + action.box_rel_pos.x,
                        current_box_position.y + action.box_rel_pos.y
                    )
                    # If the agent hasn't reached the goal, give the agent tag as 'fixed'
                    agent_tag = 'fixed'
                    box_tag = 'fixed'

                # Agent has reached the goal, but still extend its path to avoid blocking other agents
                else:
                    resulting_agent_position = current_agent_position
                    resulting_box_position = current_box_position
                    # If the agent has reached the goal, give the agent tag as 'movable'
                    agent_tag = 'movable'
                    # Box cannot move after reaching the goal
                    box_tag = 'fixed'

                ### Judge Vertex conflict ###
                if (resulting_agent_position, time_step) in positions:
                    value = positions.get((resulting_agent_position, time_step), None)
                    # Check if the value is a dictionary and has the 'tag' key
                    if isinstance(value, dict):
                        tag = value.get('tag', None)
                    else:
                        # The value is not a dictionary (possibly an integer), so 'tag' is not applicable
                        tag = None
                    other_entity_id = positions[(resulting_agent_position, time_step)]['id']
                    # Means the other conflict agent hasn't finished its goal yet
                    if tag == 'fixed':
                        # If the same conflict agent pair happens more than 3 times, then it is a deadlock
                        if conflict_counts.get((agent_id, other_entity_id), 0) < 3:
                            return Conflict(agent_id, other_entity_id, resulting_agent_position, time_step)
                        else:
                            other_agent_id = get_actual_agent_id(initial_positions, other_entity_id)
                            return MetaAgentConflict(agent_id, other_agent_id)
                    # Means the other conflict agent has finished its goal, and can try to move out the way
                    else:
                        # Can not move to the same position as other agent at current/next timestep, box, and cannot go to own/other agent's goal position
                        avoid_pos_list = {pos: agent_id for pos, agent_id in positions.items() if (pos[1] == time_step-1 or pos[1] == time_step)}
                        avoid_pos_list[(initial_positions[agent_id]['goal_position'], time_step)] = {'id': 'blocked_agent_goal'}
                        avoid_pos_list[(initial_positions[other_entity_id]['goal_position'], time_step)] = {'id': initial_positions[other_entity_id]['goal_id']}
                        return MoveAwayConflict(other_entity_id, agent_id, resulting_agent_position, avoid_pos_list, time_step-1)
                elif (resulting_box_position, time_step) in positions:
                    value = positions.get((resulting_box_position, time_step), None)
                    # Check if the value is a dictionary and has the 'tag' key
                    if isinstance(value, dict):
                        tag = value.get('tag', None)
                    else:
                        # The value is not a dictionary (possibly an integer), so 'tag' is not applicable
                        tag = None
                    other_entity_id = positions[(resulting_box_position, time_step)]['id']
                    # Means the other conflict agent hasn't finished its goal yet
                    if tag == 'fixed':
                        # If the same conflict agent pair happens more than 3 times, then it is a deadlock
                        if conflict_counts.get((box_id, other_entity_id), 0) < 3:
                            # print(f'---conflict 2 is {Conflict(box_id, other_entity_id, resulting_box_position, time_step)}')
                            return Conflict(box_id, other_entity_id, resulting_box_position, time_step)
                        else:
                            agent_id = get_actual_agent_id(initial_positions, box_id)
                            other_agent_id = get_actual_agent_id(initial_positions, other_entity_id)
                            return MetaAgentConflict(agent_id, other_agent_id)
                    # Means the other conflict agent has finished its goal, and can try to move out the way
                    else:
                        # Can not move to the same position as other agent at current/next timestep, box, and cannot go to own/other agent's goal position
                        avoid_pos_list = {pos: agent_id for pos, agent_id in positions.items() if (pos[1] == time_step-1 or pos[1] == time_step)}
                        avoid_pos_list[(initial_positions[agent_id]['goal_position'], time_step)] = {'id': 'blocked_agent_goal'}
                        avoid_pos_list[(initial_positions[other_entity_id]['goal_position'], time_step)] = {'id': initial_positions[other_entity_id]['goal_id']}
                        return MoveAwayConflict(other_entity_id, agent_id, resulting_box_position, avoid_pos_list, time_step-1)
                ### Following conflict one way ###
                # Means the other agent is moving into the current agent's/box's position at the timestep-1
                if ((current_agent_position, time_step) or (current_box_position, time_step)) in positions:
                    other_entity_id = positions[(current_agent_position, time_step)]['id']
                    # In case the other entity is a box, get its agent id, so that the later handling of the conflict can get agent directly
                    other_agent_id = get_actual_agent_id(initial_positions, other_entity_id)
                    return FollowConflict(other_agent_id, time_step-1)
                ### Following conflict the other way ###
                # Means the current agent/box is moving into the other agent's/box's position at the timestep
                positions_to_check = [(resulting_agent_position, time_step-1), (resulting_box_position, time_step-1)]
                for pos, t in positions_to_check:
                    if (pos, t) in positions:
                        other_entity_id = positions[(pos, t)]['id']
                        # eliminate the situation that the agent is moving into its own box or the other way around
                        if (other_entity_id != agent_id and other_entity_id != box_id):
                            return FollowConflict(agent_id, time_step-1)
                # Store the explored agent and box positions in the positions dictionary
                positions[(resulting_agent_position, time_step)] = {'id': agent_id, 'tag': agent_tag}
                positions[(resulting_box_position, time_step)] = {'id': box_id, 'tag': box_tag}
                current_agent_position = resulting_agent_position
                current_box_position = resulting_box_position
                time_step += 1

    return None


def solve_moveaway_conflict(node, conflict, walls):
    agent_id = conflict.ai
    blocked_agent_id = conflict.aj
    avoid_pos_list = conflict.avoid_pos_list
    agent_current_pos = conflict.current_pos
    time_step = conflict.t

    # Helper function to find a valid move action
    def find_valid_move(avoid_positions):
        for action in Action:
            if action.type == ActionType.Move:
                agent_destination = agent_current_pos + action.agent_rel_pos
                # Check if agent_destination is within the bounds of the walls matrix
                if 0 <= agent_destination.x < len(walls[0]) and 0 <= agent_destination.y < len(walls):
                    # Check if agent_destination is a wall or in avoid_positions
                    is_destination_occupied = walls[agent_destination.y][agent_destination.x] or \
                                              any(pos == agent_destination for pos, _ in avoid_positions.keys())
                    if not is_destination_occupied:
                        return action
        return None


    # Attempt round 1 to find a valid move action considering the entire avoid_pos_list
    insert_action = find_valid_move(avoid_pos_list)

    # Attempt round 2 - If no valid move action is found, try again without blocked_agent_goal
    if insert_action is None:
        filtered_avoid_pos_list = {pos: info for pos, info in avoid_pos_list.items() if 'blocked_agent_goal' not in info.values()}
        insert_action = find_valid_move(filtered_avoid_pos_list)

    if insert_action is not None:
        # Fill the solution with NoOp until the time step
        while len(node.solution[agent_id]) < time_step:
            node.solution[agent_id].append(Action.NoOp)

        # Insert the new action at the time step
        node.solution[agent_id].insert(time_step, insert_action)
        # Insert a NoOp action for the blocked agent to wait the other agent move
        node.solution[blocked_agent_id].insert(time_step, Action.NoOp)

        node.cost = cost(node.solution)
        return node
    else:
        # Attempt round 3 - If the blocked agent can move away, then it temp moves away to let the agent move away
        # Find out which action the agent takes to move into the blocked agent's position

        node = ask_blocked_agent_help(agent_id, blocked_agent_id, agent_current_pos, time_step, node, filtered_avoid_pos_list, walls)
        return node


def solve_follow_conflict(node, conflict):
    agent_id = conflict.ai
    time_step = conflict.t
    # Insert the NoOp action at the time step, so that the agent can wait for 1 step and then move
    node.solution[agent_id].insert(time_step, Action.NoOp)
    node.cost = cost(node.solution)
    # Return the new solution for the agent that needs to move away.
    return node


def solve_meta_agent_conflict(node, conflict, initial_solutions):
    agent_id = conflict.ai
    other_agent_id = conflict.aj
    # Use initial solutions for the 2 agents, as normally the initial solutions are the shortest paths
    # Use copy to avoid modifying the original solutions
    node.solution[agent_id] = initial_solutions[agent_id].copy()
    node.solution[other_agent_id] = initial_solutions[other_agent_id].copy()
    len_agent_solution = len(node.solution[agent_id])
    len_other_agent_solution = len(node.solution[other_agent_id])
    if len_agent_solution <= len_other_agent_solution:
        meta_agent_id = agent_id
        non_meta_agent_id = other_agent_id
    else:
        meta_agent_id = other_agent_id
        non_meta_agent_id = agent_id
    for t in range(len(node.solution[meta_agent_id])):
        node.solution[non_meta_agent_id].insert(t, Action.NoOp)

    return node


def solve_meta_agent_conflict(node, conflict, initial_solutions):
    agent_id = conflict.ai
    other_agent_id = conflict.aj
    # Use initial solutions for the 2 agents, as normally the initial solutions are the shortest paths
    # Use copy to avoid modifying the original solutions
    node.solution[agent_id] = initial_solutions[agent_id].copy()
    node.solution[other_agent_id] = initial_solutions[other_agent_id].copy()
    len_agent_solution = len(node.solution[agent_id])
    len_other_agent_solution = len(node.solution[other_agent_id])
    if len_agent_solution <= len_other_agent_solution:
        meta_agent_id = agent_id
        non_meta_agent_id = other_agent_id
    else:
        meta_agent_id = other_agent_id
        non_meta_agent_id = agent_id
    for t in range(len(node.solution[meta_agent_id])):
        node.solution[non_meta_agent_id].insert(t, Action.NoOp)

    return node
