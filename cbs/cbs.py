import itertools
import sys
import time

from astar import astar
from cbs.node import Node

from domain.action import Action, ActionType
from domain.conflict import Conflict, MoveAwayConflict,FollowConflict, MetaAgentConflict
from domain.constraint import Constraint
from domain.position import Position
from queue import PriorityQueue
from htn.htn_resolver import HTNResolver
from st_astar import space_time_a_star
from state import State

# Create a counter that will serve as a tiebreaker
tiebreaker = itertools.count()

def conflict_based_search(current_state: State, round):
    print(f"\n=============CBS-begin===============", file=sys.stderr)
    root = Node()
    root.constraints = set()
    initial_positions = initialize_initial_positions(current_state, round)

    for agent in current_state.agents:
        if(round[agent.value].goal_uid == None):
            continue
        relaxed_state = current_state.from_agent_perspective(agent.value)
        plan = astar(relaxed_state, round[agent.value])
        root.solution[agent.value] = plan

    print(f'CBS solution: ', root.solution, file=sys.stderr)

    root.cost = cost(root.solution)
    frontier = PriorityQueue()
    count_next = next(tiebreaker)
    frontier.put((count_next, root))

    conflict_special_solvers = {
        MoveAwayConflict: solve_moveaway_conflict,
        FollowConflict: solve_follow_conflict,
    }

    # Store the initial solution to be used in the long corridor situation
    initial_solutions = root.solution
    # Create a dictionary to store the number of conflicts for each agent pair
    conflict_counts = {}

    while not frontier.empty():
        tiebreaker_value, node = frontier.get()
        conflict = find_first_conflict(node.solution, initial_positions, conflict_counts)
        if conflict is None:
            print(f"No conflict found. We are extracting plan... -> {node.solution}", file=sys.stderr)
            executable_plan = merge_plans(current_state, node.solution, round)
            print(f"=============CBS-end===============\n", file=sys.stderr)
            return executable_plan

        handler = conflict_special_solvers.get(type(conflict), None)

        if(handler):
            new_node = handler(node.copy(), conflict)
            # Immediately check if the special conflict is solved,
            # else if the cost is more than the other nodes, the new solutions won't be prioritized
            # The special conflict is supposed to be solved in one step,
            # if not, then abandon the solution and try to solve the conflict in the next iteration
            new_conflict = find_first_conflict(new_node.solution, initial_positions, conflict_counts)
            print(f"=======================new_conflict=================={new_conflict}", file=sys.stderr)
            if new_conflict is None:
                print(f"No conflict found. We are extracting plan... -> {new_node.solution}", file=sys.stderr)
                executable_plan = merge_plans(current_state, new_node.solution, round)
                print(f"=============CBS-end===============\n", file=sys.stderr)
                return executable_plan
            else:
                print('-----------Special conflict not solved, adding to frontier.----------')
                new_node.cost = cost(new_node.solution)
                if new_node.cost < sys.maxsize:
                    count_next = next(tiebreaker)
                    frontier.put((count_next, new_node))
                continue
        elif isinstance(conflict, MetaAgentConflict):
            m = node.copy()
            new_node = solve_meta_agent_conflict(node, conflict, initial_solutions)
            print(f"---new_node.solution--{new_node.solution}", file=sys.stderr)
            new_conflict = find_first_conflict(new_node.solution, initial_positions, conflict_counts)
            if new_conflict is None:
                print(f"No conflict found. We are extracting plan... -> {new_node.solution}", file=sys.stderr)
                executable_plan = merge_plans(current_state, new_node.solution, round)
                print(f"=============CBS-end===============\n", file=sys.stderr)
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
            print(f'=================Conflict count: ===================\n{conflict_counts}', file=sys.stderr)
            resolve_conflict(node, conflict, current_state, round, frontier)

    print(f"CBS cannot resolve conflict", file=sys.stderr)
    print(f"=============CBS-end===============\n", file=sys.stderr)
    return None

def resolve_conflict(node, conflict, current_state, round, frontier):
    for agent_uid, task in round.items():
        #round--{0: Task(box_uid=0 goal_uid=1), 1: Task(box_uid=1 goal_uid=0)}
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
        relaxed_state = current_state.from_agent_perspective(agent_uid)
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
        print('No solution found from astar.', file=sys.stderr)
        return None
    # the format of solution is {agent_id: [action1,action2,action3], agent_id: [action1,action2,action3]}
    # print(f"---initial_positions--{initial_positions}", file=sys.stderr)

    # Create a dictionary to track positions of each entity at each time step
    positions = {}
    avoid_pos_list = {}
    # Find the maximum path length to know how many time steps to check
    max_path_length = max(len(path) for path in solution.values())
    # print(f"---max_path_length--{max_path_length}", file=sys.stderr)

    # sort agent by their path length, so that we always start from short path
    sorted_agents = sorted(solution.items(), key=lambda item: len(item[1]))
    # print(f"---sorted_agents--{sorted_agents}", file=sys.stderr)
    for agent_id, path in sorted_agents:
        # print(f"---agent_id--{agent_id}", file=sys.stderr)
        # print(f'---box is--{initial_positions[agent_id]["box_id"]}', file=sys.stderr)
        # print(f'---path is--{path}', file=sys.stderr)

        # agent-agent conflict
        if initial_positions[agent_id]['box_id'] is None:
            current_position = initial_positions[agent_id]['agent_position']
            # print(f"---current_position--{current_position}", file=sys.stderr)
            # print(f"---agent_id--{agent_id}", file=sys.stderr)
            # print(f"---path--{path}", file=sys.stderr)
            # Add initial position to the positions dictionary as time step 0
            positions[(current_position, 0)] = agent_id
            time_step = 1 # Start the first step at 1
            # Loop over the max path length to avoid situation that one agent reach the goal and stop moving, but still block the other agents
            while time_step < len(path)+1:
                # if time_step < len(path)+1:
                    # get the action at the current time step, begining from path[0].
                action = path[time_step-1]
                # Calculate the resulting position of the agent after the action
                resulting_position = Position(
                    current_position.x + action.agent_rel_pos.x,
                    current_position.y + action.agent_rel_pos.y
                )
                # print(f"---resulting_position--{resulting_position}",file=sys.stderr)
                if (resulting_position, time_step) in positions:
                    # Conflict detected, return information about the conflict
                    other_agent_id = positions[(resulting_position, time_step)]
                    print(f"---Conflict--{Conflict(agent_id, other_agent_id, resulting_position, time_step)}", file=sys.stderr)
                    return Conflict(agent_id, other_agent_id, resulting_position, time_step)
                ### Following conflict one way ###
                # Other agent is moving into the current agent's position at the timestep
                if (current_position, time_step) in positions:
                    other_agent_id = positions[(current_position, time_step)]
                    # remove situation that the agent id is itself because of NoOp action
                    if other_agent_id != agent_id:
                        print(f"---Follow Conflict Agent 1--{FollowConflict(other_agent_id, time_step-1)}",file=sys.stderr)
                        return FollowConflict(other_agent_id, time_step-1)
                ### Following conflict the other way ###
                # Current agent is moving into the other agent's position at the timestep
                if (resulting_position, time_step-1) in positions:
                    other_agent_id = positions[(resulting_position, time_step-1)]
                    # remove situation that the agent id is itself because of NoOp action
                    if other_agent_id != agent_id:
                        print(f"---Follow Conflict Agent 2--{FollowConflict(agent_id, time_step-1)}",file=sys.stderr)
                        return FollowConflict(agent_id, time_step-1)

                # update the position of the agent at specific time step
                positions[(resulting_position, time_step)] = agent_id
                #print(f"---positions--{positions}")
                # Update the current position of the agent
                current_position = resulting_position
                time_step += 1

        # agent-box conflict
        # box-box conflict
        else:
            current_agent_position = initial_positions[agent_id]['agent_position']
            box_id = initial_positions[agent_id]['box_id']
            # print(f"--box_id--{box_id}", file=sys.stderr)
            current_box_position = initial_positions[agent_id]['box_position']
            # print(f"---current_agent_position--{current_agent_position}", file=sys.stderr)
            # print(f"---current_box_position--{current_box_position}", file=sys.stderr)
            # print(f"---box_id--{box_id}", file=sys.stderr)

            # Add initial positions to the positions dictionary as time step 0, and as resulting positions
            positions[(current_agent_position, 0)] = {'id': agent_id, 'tag': 'fixed'}
            positions[(current_box_position, 0)] = {'id': box_id, 'tag': 'fixed'}
            #print(f"---positions--{positions}",file=sys.stderr)

            time_step = 1
            # Iterate for max_path_length steps
            for time_step in range(1, max_path_length + 1):
                # print(f"---time_step--{time_step}", file=sys.stderr)
                # Agent is within its path
                if time_step < len(path)+1:
                    action = path[time_step - 1]
                    #print(f"---action--{action}",file=sys.stderr)
                    resulting_agent_position = Position(
                        current_agent_position.x + action.agent_rel_pos.x,
                        current_agent_position.y + action.agent_rel_pos.y
                    )
                    resulting_box_position = Position(
                        current_box_position.x + action.box_rel_pos.x,
                        current_box_position.y + action.box_rel_pos.y
                    )
                    #print(f"---resulting_agent_position--{resulting_agent_position}",file=sys.stderr)
                    #print(f"---resulting_box_position--{resulting_box_position}",file=sys.stderr)
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
                    tag = positions[(resulting_agent_position, time_step)]['tag']
                    other_entity_id = positions[(resulting_agent_position, time_step)]['id']
                    # Means the other conflict agent hasn't finished its goal yet
                    if tag == 'fixed':
                        # If the same conflict agent pair happens more than 3 times, then it is a deadlock
                        if conflict_counts.get((agent_id, other_entity_id), 0) <= 3:
                            # print(f'conflict_counts original--{conflict_counts}',file=sys.stderr)
                            # print(f'agent_id--{agent_id}',file=sys.stderr)
                            # print(f'other_entity_id--{other_entity_id}',file=sys.stderr)
                            # print(f'conflict_counts--{conflict_counts.get((agent_id, other_entity_id), 0)}',file=sys.stderr)
                            # print(f"---Conflict 1--{Conflict(agent_id, other_entity_id, resulting_agent_position, time_step)}",file=sys.stderr)
                            return Conflict(agent_id, other_entity_id, resulting_agent_position, time_step)
                        else:
                            other_agent_id = get_actual_agent_id(initial_positions, other_entity_id)
                            return MetaAgentConflict(agent_id, other_agent_id)
                    # Means the other conflict agent has finished its goal, and can try to move out the way
                    else:
                        # Can not move to the same position as other agent at current/next timestep, box, and cannot go to own/other agent's goal position
                        avoid_pos_list = {pos: agent_id for pos, agent_id in positions.items() if (pos[1] == time_step-1 or pos[1] == time_step)}
                        avoid_pos_list[(initial_positions[agent_id]['goal_position'], time_step)] = {'id': initial_positions[agent_id]['goal_id']}
                        avoid_pos_list[(initial_positions[other_entity_id]['goal_position'], time_step)] = {'id': initial_positions[other_entity_id]['goal_id']}
                        #print(f'---avoid_pos_list 1--{avoid_pos_list}',file=sys.stderr)
                        print(f"---MoveAwayConflict 1 --{MoveAwayConflict(other_entity_id, agent_id, resulting_agent_position, avoid_pos_list, time_step-1)}",file=sys.stderr)
                        return MoveAwayConflict(other_entity_id, agent_id, resulting_agent_position, avoid_pos_list, time_step-1)
                elif (resulting_box_position, time_step) in positions:
                    tag = positions[(resulting_box_position, time_step)]['tag']
                    other_entity_id = positions[(resulting_box_position, time_step)]['id']
                    # Means the other conflict agent hasn't finished its goal yet
                    if tag == 'fixed':
                        # If the same conflict agent pair happens more than 3 times, then it is a deadlock
                        if conflict_counts.get((box_id, other_entity_id), 0) <= 3:
                            # print(f'conflict_counts original--{conflict_counts}',file=sys.stderr)
                            # print(f'agent_id--{agent_id}',file=sys.stderr)
                            # print(f'other_entity_id--{other_entity_id}',file=sys.stderr)
                            # print(f'conflict_counts--{conflict_counts.get((agent_id, other_entity_id), 0)}',file=sys.stderr)
                            # print(f"---Conflict 2--{Conflict(box_id, other_entity_id, resulting_box_position, time_step)}",file=sys.stderr)
                            return Conflict(box_id, other_entity_id, resulting_box_position, time_step)
                        else:
                            agent_id = get_actual_agent_id(initial_positions, box_id)
                            other_agent_id = get_actual_agent_id(initial_positions, other_entity_id)
                            return MetaAgentConflict(agent_id, other_agent_id)
                    # Means the other conflict agent has finished its goal, and can try to move out the way
                    else:
                        # Can not move to the same position as other agent at current/next timestep, box, and cannot go to own/other agent's goal position
                        avoid_pos_list = {pos: agent_id for pos, agent_id in positions.items() if (pos[1] == time_step-1 or pos[1] == time_step)}
                        avoid_pos_list[(initial_positions[agent_id]['goal_position'], time_step)] = {'id': initial_positions[agent_id]['goal_id']}
                        avoid_pos_list[(initial_positions[other_entity_id]['goal_position'], time_step)] = {'id': initial_positions[other_entity_id]['goal_id']}
                        #print(f'---avoid_pos_list 2--{avoid_pos_list}',file=sys.stderr)
                        print(f"---MoveAwayConflict 2 --{MoveAwayConflict(other_entity_id, agent_id, resulting_box_position, avoid_pos_list, time_step-1)}",file=sys.stderr)
                        return MoveAwayConflict(other_entity_id, agent_id, resulting_box_position, avoid_pos_list, time_step-1)
                ### Following conflict one way ###
                # Means the other agent is moving into the current agent's/box's position at the timestep-1
                if ((current_agent_position, time_step) or (current_box_position, time_step)) in positions:
                    # print(f'---agent_id--{agent_id}',file=sys.stderr)
                    # print(f'---current_agent_position--{current_agent_position}',file=sys.stderr)
                    # print(f'---current_box_position--{current_box_position}',file=sys.stderr)
                    other_entity_id = positions[(current_agent_position, time_step)]['id']
                    #print(f'---other_entity_id--{other_entity_id}',file=sys.stderr)
                    # In case the other entity is a box, get its agent id, so that the later handling of the conflict can get agent directly
                    other_agent_id = get_actual_agent_id(initial_positions, other_entity_id)
                    print(f"---Follow Conflict 1--{FollowConflict(other_agent_id, time_step-1)}",file=sys.stderr)
                    return FollowConflict(other_agent_id, time_step-1)
                ### Following conflict the other way ###
                # Means the current agent/box is moving into the other agent's/box's position at the timestep
                positions_to_check = [(resulting_agent_position, time_step-1), (resulting_box_position, time_step-1)]
                for pos, t in positions_to_check:
                    if (pos, t) in positions:
                        other_entity_id = positions[(pos, t)]['id']
                        #print(f'###other_entity_id 1--{other_entity_id}',file=sys.stderr)
                        # eliminate the situation that the agent is moving into its own box or the other way around
                        if (other_entity_id != agent_id and other_entity_id != box_id):
                            # print(f'---other_entity_id--{other_entity_id}',file=sys.stderr)
                            # print(f'---resulting_agent_position--{resulting_agent_position}',file=sys.stderr)
                            # print(f'---resulting_box_position--{resulting_box_position}',file=sys.stderr)
                            print(f"---Follow Conflict 2--{FollowConflict(agent_id, time_step-1)}",file=sys.stderr)
                            return FollowConflict(agent_id, time_step-1)
                # Store the explored agent and box positions in the positions dictionary
                positions[(resulting_agent_position, time_step)] = {'id': agent_id, 'tag': agent_tag}
                positions[(resulting_box_position, time_step)] = {'id': box_id, 'tag': box_tag}
                #print(f"---positions--{positions}",file=sys.stderr)
                current_agent_position = resulting_agent_position
                current_box_position = resulting_box_position
                time_step += 1

    return None


def merge_plans(current_state, solutions, round):
    """
    Merge the individual plans of the agents into a single executable plan.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: A list of joint actions that represents the executable plan.
    """
    # Initialize the merged plan
    merged_plan = []
    # Find the maximum length of the individual agent plans
    max_length = max(len(plan) for plan in solutions.values())


    # For each agent, get the action at the current step or use NoOp if the plan is shorter
    sorted_agents = sorted(current_state.agents, key=lambda a: a.value)
    for step in range(max_length):
        joint_action = []
        for agent in sorted_agents:
            if(agent.value in solutions.keys()):
                solution = solutions[agent.value]
                if step < len(solution):
                    action = solution[step]
                else:
                    # Assuming NoOp is represented as None or a specific NoOp action
                    action = Action.NoOp
                joint_action.append(action)
            else:
                joint_action.append(Action.NoOp)
        # Append the joint action to the merged plan
        merged_plan.append(joint_action)
    print(f"---merged_plan--{merged_plan}", file=sys.stderr)

    for agent_uid, task in round.items():
        if(HTNResolver.completed_tasks.get(agent_uid) is None):
            HTNResolver.completed_tasks[agent_uid] = []
        HTNResolver.completed_tasks[agent_uid].append(task)
    # print(f"round--{round}", file=sys.stderr)
    return merged_plan

def solve_moveaway_conflict(node, conflict):
    agent_id = conflict.ai
    blocked_agent_id = conflict.aj
    avoid_pos_list = conflict.avoid_pos_list
    agent_current_pos = conflict.current_pos
    time_step = conflict.t

    # Move E, W, N, S
    for action in Action:
        if action.type == ActionType.Move:
            agent_destination = agent_current_pos + action.agent_rel_pos
            # Check if agent_destination is in avoid_pos_list
            is_destination_occupied = any(pos == agent_destination for pos, _ in avoid_pos_list.keys())
            if is_destination_occupied:
                continue
            else:
                # print(f"---agent_destination--{agent_destination}", file=sys.stderr)
                # print(f"---action--{action}", file=sys.stderr)
                insert_action = action
                break
    # print(f"---insert_action--{insert_action}",file=sys.stderr)
    # print(f"---node.solution[agent_id]--{node.solution[agent_id]}",file=sys.stderr)

    # Fill the solution with NoOp until the time step
    while len(node.solution[agent_id]) < time_step:
        node.solution[agent_id].append(Action.NoOp)

    # Insert the new action at the time step
    node.solution[agent_id].insert(time_step, insert_action)
    # Insert a NoOp action for the blocked agent to wait the other agent move
    node.solution[blocked_agent_id].insert(time_step, Action.NoOp)

    node.cost = cost(node.solution)
    return node


def solve_follow_conflict(node, conflict):
    agent_id = conflict.ai
    time_step = conflict.t
    # Insert the NoOp action at the time step, so that the agent can wait for 1 step and then move
    node.solution[agent_id].insert(time_step, Action.NoOp)
    node.cost = cost(node.solution)
    # Return the new solution for the agent that needs to move away.
    return node


def get_actual_agent_id(initial_positions, entity_id):
    # If the entity_id is a box, return its corresponding agent_id
    for agent_id, pos in initial_positions.items():
        if pos['box_id'] == entity_id:
            return agent_id
    # If the entity_id is already an agent_id, return it as is
    return agent_id


def solve_meta_agent_conflict(node, conflict, initial_solutions):
    agent_id = conflict.ai
    other_agent_id = conflict.aj
    # Use initial solutions for the 2 agents, as normally the initial solutions are the shortest paths
    node.solution[agent_id] = initial_solutions[agent_id]
    node.solution[other_agent_id] = initial_solutions[other_agent_id]
    len_agent_solution = len(node.solution[agent_id])
    len_other_agent_solution = len(node.solution[other_agent_id])
    if len_agent_solution < len_other_agent_solution:
        meta_agent_id = agent_id
        non_meta_agent_id = other_agent_id
    else:
        meta_agent_id = other_agent_id
        non_meta_agent_id = agent_id

    for t in range(len(node.solution[meta_agent_id])):
        node.solution[non_meta_agent_id].insert(t, Action.NoOp)

    print(f"---node.solution[non_meta_agent_id]--{node.solution[non_meta_agent_id]}",file=sys.stderr)
    return node
