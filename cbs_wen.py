import itertools
import sys
import time

from astar import astar
from cbs.node import Node

from domain.action import Action, ActionType
from domain.conflict import Conflict, MoveAwayConflict
from domain.constraint import Constraint
from domain.position import Position
from queue import PriorityQueue
from st_astar import space_time_a_star
from state import State

# Create a counter that will serve as a tiebreaker
tiebreaker = itertools.count()

def conflict_based_search(current_state: State, round):
    root = Node()
    root.constraints = set()
    initial_positions = {}
    #round--{0: Task(box_uid=-1 goal_uid=1), 1: Task(box_uid=-1 goal_uid=0)}
    print(f"--round--{round}", file=sys.stderr)
    for agent_uid, task in round.items():
        agent = current_state.get_agent_by_uid(agent_uid)
        box = current_state.get_box_by_uid(task.box_uid)
        initial_positions[agent.uid] = {
                'agent_position': agent.pos,
                'box_id': box.uid if box else None,
                'box_position': box.pos if box else None
            }
    print(f"---initial_positions--{initial_positions}", file=sys.stderr)
    
    for agent in current_state.agents:
        # print(f"---agent--{agent}", file=sys.stderr)
        # print(f"--current_state--{current_state}", file=sys.stderr)
        # agent = current_state.get_agent_by_uid(agent.uid)
        # solution--[[MoveE, MoveN], [MoveE, MoveN], [MoveE, MoveN]]
        # Convert solution list to dictionary {agent.uid: [actions]}
        # Take out the action of each agent separately and put it under the corresponding agent.uid
        # The actions here include the first action of all agents, so the first action needs to be taken
        # Suppose we have two agents, agent1’s action is MoveE, and agent2’s action is MoveN.
        # then root.solution = {0: [MoveE], 1: [MoveN]}
        # In this way, you need to loop through the actions in the solution and put them under the corresponding agent.uid.
        # root.solution[agent.uid] = [actions[agent.uid] for actions in solution]
        # print(f"---root.solution--{root.solution}", file=sys.stderr)
        # root.solution = {0: [MoveE,MoveE,MoveE], 1: [MoveN,MoveN,MoveN]}
    
        relaxed_state = current_state.from_agent_perspective(agent.uid)
        print(f"---relaxed_state.boxes--{relaxed_state.boxes} {agent.uid}", file=sys.stderr)
        plan = astar(relaxed_state, round[agent.uid])
        root.solution[agent.uid] = plan
    
    print(f'solution: ', root.solution, file=sys.stderr)
        
    root.cost = cost(root.solution)
    frontier = PriorityQueue()
    count_next = next(tiebreaker)
    frontier.put((root.cost, count_next, root))
    while not frontier.empty():
        node_cost, tiebreaker_value, node = frontier.get()
        conflict = find_first_conflict(node.solution, initial_positions)
        if conflict is None:
            print(f"No conflict found. We are extracting plan... -> {node.solution}", file=sys.stderr)
            executable_plan = merge_plans(node.solution,round)
            return executable_plan
        elif isinstance(conflict, MoveAwayConflict):
            print(f"---Moveaway conflict--{conflict}", file=sys.stderr)
            m = node.copy()
            moveaway_agent_id = conflict.ai
            blocked_agent_id = conflict.aj
            m.solution[moveaway_agent_id], m.solution[blocked_agent_id] = solve_moveaway_conflict(node, conflict)
            # Check if there is conflict for the new plans, if not, merge the plans, else continue
            conflict = find_first_conflict(m.solution, initial_positions)
            if conflict is None:
                print(f"No conflict found. We are extracting plan... -> {m.solution}", file=sys.stderr)
                executable_plan = merge_plans(m.solution,round)
                return executable_plan
            else:
                continue
        else:
            print(f"---conflict--{conflict}", file=sys.stderr)
            # TODO : check how to merge-------------------------------------
            # conflict between agent - agent, agent - box
            for agent_uid, task in round.items():
                if(agent_uid in [conflict.ai, conflict.aj]):

                
                    m = node.copy()
                    
                    m.constraints.add(Constraint(agent_uid, Position(conflict.pos.x, conflict.pos.y), conflict.t))
                    m.constraints.add(Constraint(agent_uid, Position(conflict.pos.x, conflict.pos.y), conflict.t+1))
                    m.constraints.add(Constraint(agent_uid, Position(conflict.pos.x, conflict.pos.y), conflict.t+2))
                    relaxed_state = current_state.from_agent_perspective(agent_uid)
                    st_solution = space_time_a_star(relaxed_state, m.constraints, round[agent_uid])
                    print(f"---st_solution--{st_solution}", file=sys.stderr)
                    m.solution[agent_uid] = st_solution
                        
                    #m.solution[agent_id] = space_time_a_star(current_state, m.constraints, round)
                    m.node_cost = cost(m.solution)
                    if m.node_cost < sys.maxsize:
                        count_next = next(tiebreaker)
                        frontier.put((m.node_cost, count_next, m))

                # conflict between box - box
                else:
                    box = current_state.get_box_by_uid(task.box_uid)
                    print(f"box_uid {box.uid}")
                    print(f"conflict.ai {conflict.ai}")
                    print(f"conflict.aj {conflict.aj}")
                    if(box.uid in [conflict.ai, conflict.aj]):
                #for box.uid in [conflict.ai, conflict.aj]:
                        m = node.copy()
                        m.constraints.add(Constraint(box.uid, Position(conflict.pos.x, conflict.pos.y), conflict.t))
                        m.constraints.add(Constraint(box.uid, Position(conflict.pos.x, conflict.pos.y), conflict.t+1))
                        m.constraints.add(Constraint(box.uid, Position(conflict.pos.x, conflict.pos.y), conflict.t+2))
                        relaxed_state = current_state.from_agent_perspective(agent_uid)
                        m.solution[agent_uid] = space_time_a_star(relaxed_state, m.constraints, round[agent_uid])
                        m.node_cost = cost(m.solution)
                        if m.node_cost < sys.maxsize:
                            count_next = next(tiebreaker)
                            frontier.put((m.node_cost, count_next, m))

        
        
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

def find_first_conflict(solution, initial_positions):
    """
    Find the first conflict in the solution.

    :param solution: A dictionary mapping entity(agent/box) IDs to their respective paths (lists of actions).
    :return: A conflict object with details about the conflict, or None if no conflict is found.
    """
    if not solution:
        print('No solution found from astar.', file=sys.stderr)
        return None
    # the format of solution is {agent.uid: [action1,action2,action3], agent.uid: [action1,action2,action3]}
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
            time_step = 1 # Start the first step at 1
            # Loop over the max path length to avoid situation that one agent reach the goal and stop moving, but still block the other agents
            while time_step < max_path_length:
                if time_step < len(path)+1:
                    # action_list = path[time_step - 1]
                    # print(f"---action--{action_list}")
                    # print(f"--path--{path}")
                    # action = action_list[0]
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
                        # print(f"---Conflict--{Conflict(agent_id, other_agent_id, resulting_position, time_step)}", file=sys.stderr)
                        return Conflict(agent_id, other_agent_id, resulting_position, time_step)

                # If the agent has reached the goal, still extend its path to avoid blocking other agents
                else:
                    resulting_position = current_position
                    if (resulting_position, time_step) in positions:
                        # Conflict detected, return information about the conflict
                        other_agent_id = positions[(resulting_position, time_step)]
                        # print(f"---MoveAwayConflict--{MoveAwayConflict(agent_id, resulting_agent_position, avoid_pos_list, time_step-1)}", file=sys.stderr)
                        return MoveAwayConflict(agent_id, resulting_agent_position, avoid_pos_list, time_step-1)

                positions[(resulting_position, time_step)] = agent_id
                # print(f"---positions--{positions}", file=sys.stderr)
                # Update the current position of the agent
                current_position = resulting_position
                time_step += 1

        # agent-box conflict
        # box-box conflict
        else:
            current_agent_position = initial_positions[agent_id]['agent_position']
            box_id = initial_positions[agent_id]['box_id']
            current_box_position = initial_positions[agent_id]['box_position']
            # print(f"---current_agent_position--{current_agent_position}", file=sys.stderr)
            # print(f"---current_box_position--{current_box_position}", file=sys.stderr)
            # print(f"---box_id--{box_id}", file=sys.stderr)
            time_step = 1
            # Iterate for max_path_length steps
            for time_step in range(1, max_path_length + 1):
                # print(f"---time_step--{time_step}", file=sys.stderr)
                # Agent is within its path
                if time_step < len(path)+1:
                    # action_list = path[time_step - 1]
                    # action = action_list[0]
                    # print(f"---action--{action}")
                    action = path[time_step-1]
                    resulting_agent_position = Position(
                        current_agent_position.x + action.agent_rel_pos.x,
                        current_agent_position.y + action.agent_rel_pos.y
                    )
                    resulting_box_position = Position(
                        current_box_position.x + action.box_rel_pos.x,
                        current_box_position.y + action.box_rel_pos.y
                    )
                    # print(f"---resulting_agent_position--{resulting_agent_position}", file=sys.stderr)
                    # print(f"---resulting_box_position--{resulting_box_position}", file=sys.stderr)
                    if (resulting_agent_position, time_step) in positions:
                        other_entity_id = positions[(resulting_agent_position, time_step)]
                        # print(f"---Conflict--{Conflict(agent_id, other_entity_id, resulting_agent_position, time_step)}", file=sys.stderr)
                        return Conflict(agent_id, other_entity_id, resulting_agent_position, time_step)
                    elif (resulting_box_position, time_step) in positions:
                        other_entity_id = positions[(resulting_box_position, time_step)]
                        # print(f"---Conflict--{Conflict(box_id, other_entity_id, resulting_box_position, time_step)}", file=sys.stderr)
                        return Conflict(box_id, other_entity_id, resulting_box_position, time_step)

                # Agent has reached the goal, but still extend its path to avoid blocking other agents
                else:
                    resulting_agent_position = current_agent_position
                    resulting_box_position = current_box_position
                    if (resulting_agent_position, time_step) in positions:
                        other_entity_id = positions[(resulting_agent_position, time_step)]
                        # print(f'###### positions--{positions}########', file=sys.stderr)
                        avoid_pos_list = {pos: agent_id for pos, agent_id in positions.items() if pos[1] >= time_step-1}
                        # print(f'---avoid_pos_list 1--{avoid_pos_list}', file=sys.stderr)
                        # print(f"---MoveAwayConflict 1 --{MoveAwayConflict(agent_id, resulting_agent_position, avoid_pos_list, time_step-1)}", file=sys.stderr)
                        return MoveAwayConflict(agent_id, resulting_agent_position, avoid_pos_list, time_step-1)
                    # box cannot move away when it reaches the goal, so need to return normal conflict for the other agent to replan
                    elif (resulting_box_position, time_step) in positions:
                        other_entity_id = positions[(resulting_box_position, time_step)]
                        # print(f"---Conflict--{Conflict(box_id, other_entity_id, resulting_box_position, time_step)}", file=sys.stderr)
                        return Conflict(box_id, other_entity_id, resulting_box_position, time_step)
                    # print(f"---resulting_agent_position in else--{resulting_agent_position}", file=sys.stderr)
                    # print(f"---resulting_box_position in else--{resulting_box_position}", file=sys.stderr)

                positions[(resulting_agent_position, time_step)] = agent_id
                positions[(resulting_box_position, time_step)] = box_id
                # print(f"---positions--{positions}", file=sys.stderr)
                current_agent_position = resulting_agent_position
                current_box_position = resulting_box_position
                time_step += 1

    return None


# def merge_plans(plans):
#     """
#     Merge the individual plans of the agents into a single executable plan.

#     :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
#     :return: A list of joint actions that represents the executable plan.
#     """
#     # Initialize the merged plan
#     merged_plan = []
#     # Find the maximum length of the individual agent plans
#     max_length = max(len(plan) for plan in plans.values())

#     # Iterate over each time step
#     for step in range(max_length):
#         joint_action = []

#         # For each agent, get the action at the current step or use NoOp if the plan is shorter
#         for agent_id, plan in plans.items():
#             if step < len(plan):
#                 #action = plan[step][0]  # Each action is a list, take the first element
#                 action = plan[step]
#             else:
#                 # Assuming NoOp is represented as None or a specific NoOp action
#                 action = Action.NoOp
#             joint_action.append(action)

#         # Append the joint action to the merged plan
#         merged_plan.append(joint_action)
#     print(f"---merged_plan--{merged_plan}", file=sys.stderr)
#     return merged_plan

def merge_plans(plans, round):
    """
    Merge the individual plans of the agents into a single executable plan.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: A list of joint actions that represents the executable plan.
    """
    # Initialize the merged plan
    merged_plan = []
    # Find the maximum length of the individual agent plans
    max_length = max(len(plan) for plan in plans.values())

    # Iterate over each time step
    for step in range(max_length):
        joint_action = []

        # For each agent, get the action at the current step or use NoOp if the plan is shorter
        for agent_id, plan in plans.items():
            if step < len(plan):
                #action = plan[step][0]  # Each action is a list, take the first element
                action = plan[step]
            else:
                # Assuming NoOp is represented as None or a specific NoOp action
                action = Action.NoOp
            joint_action.append(action)

        # Append the joint action to the merged plan
        merged_plan.append(joint_action)
    print(f"---merged_plan--{merged_plan}", file=sys.stderr)
    for task in round.values():
        # print(f"---task--{task}", file=sys.stderr)
        task.is_completed = True
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
                insert_action = [action]
                break
    # print(f"---insert_action--{insert_action}")
    # print(f"---node.solution[agent_id]--{node.solution[agent_id]}")

    # Fill the solution with NoOp until the time step
    while len(node.solution[agent_id]) < time_step:
        node.solution[agent_id].append([Action.NoOp])

    # Insert the new action at the time step
    node.solution[agent_id].insert(time_step, insert_action)
    # Insert a NoOp action for the blocked agent to wait the other agent move
    node.solution[blocked_agent_id].insert(time_step, [Action.NoOp])

    # Return the new solution for the agent that needs to move away.
    return node.solution[agent_id], node.solution[blocked_agent_id]
