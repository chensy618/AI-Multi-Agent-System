import itertools
import sys

from astar import astar
from cbs.node import Node

from domain.action import Action, ActionType
from domain.conflict import Conflict, MoveAwayConflict
from domain.constraint import Constraint
from domain.position import Position
from queue import PriorityQueue
from st_astar import space_time_a_star

# Create a counter that will serve as a tiebreaker
tiebreaker = itertools.count()

def conflict_based_search(problem_list):
    root = Node()
    root.constraints = []
    initial_positions = {}

    # Get initial positions
    for problem in problem_list:

        box_by_color = {box.color: box for box in problem.boxes}
        # Iterate over agents, assuming that not all agents may have a corresponding box
        for agent in problem.agents:
            # Find the corresponding box if it exists using the color mapping
            box = box_by_color.get(agent.color, None)

            # Create a dictionary with the agent and box information
            # If there is no corresponding box, use None for the box ID and position
            initial_positions[agent.id] = {
                'agent_position': agent.pos,
                'box_id': box.id if box else None,
                'box_position': box.pos if box else None
            }
            root.solution[agent.id] = astar(problem)

    print(f"---initial_positions--{initial_positions}")
    # ---initial_positions--[(0, Position(x=3, y=1), 'A', Position(x=3, y=2)), (1, Position(x=5, y=3), 'B', Position(x=4, y=3))]

    root.cost = cost(root.solution)
    frontier = PriorityQueue()
    count_next = next(tiebreaker)
    frontier.put((root.cost, count_next, root))
    while not frontier.empty():
        node_cost, tiebreaker_value, node = frontier.get()

        # Find first conflict in the solutions
        conflict = find_first_conflict(node.solution, initial_positions)
        if conflict is None:
            print('I am here, no conflict found.')
            executable_plan = merge_plans(node.solution)
            return executable_plan
        # Special handling for MoveAwayConflict
        elif isinstance(conflict, MoveAwayConflict):
            print(f"---Moveaway conflict--{conflict}")
            m = node.copy()
            moveaway_agent_id = conflict.ai
            m.solution[moveaway_agent_id] = solve_moveaway_conflict(node, conflict)
            m.node_cost = cost(m.solution)
            print(f"---m.node_cost--{m.node_cost}")
            if m.node_cost < sys.maxsize:
                count_next = next(tiebreaker)
                frontier.put((m.node_cost, count_next, m))  # Include the tiebreaker in the tuple

        else:
            for problem in problem_list:
                for agent in problem.agents:
                    if agent.id in [conflict.ai, conflict.aj]:
                        print(f"----------------------------agent.id--{agent.id}")
                        m = node.copy()
                        m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t))
                        m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+1))
                        m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+2))
                        print(f"---m.constraints--{m.constraints}")
                        m.solution[agent.id] = space_time_a_star(problem, m.constraints)
                        print(f"---m.solution--{m.solution[agent.id]}")
                        m.node_cost = cost(m.solution)
                        print(f"---m.node_cost--{m.node_cost}")
                        # When putting a node into the priority queue, include the tiebreaker
                        # To be able to solve the issue when the costs are equal
                        if m.node_cost < sys.maxsize:
                            count_next = next(tiebreaker)
                            frontier.put((m.node_cost, count_next, m))  # Include the tiebreaker in the tuple

                    # If agent is not involved in the conflict, then try to check box conflicts
                    else:
                        # TODO: Optimize here when there is new problem structure
                        # Get corresponding box id
                        box = next((b for b in problem.boxes if b.color == agent.color), None)
                        print(f"-------------------------------box.id--{box.id}")
                        print(f"-------------------------------agent.id--{agent.id}")
                        if box.id in [conflict.ai, conflict.aj]:
                            m = node.copy()
                            m.constraints.append(Constraint(box.id, Position(conflict.pos.x, conflict.pos.y), conflict.t))
                            m.constraints.append(Constraint(box.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+1))
                            m.constraints.append(Constraint(box.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+2))
                            print(f"---m.constraints--{m.constraints}")
                            m.solution[agent.id] = space_time_a_star(problem, m.constraints)
                            print(f"---m.solution--{m.solution[agent.id]}")
                            m.node_cost = cost(m.solution)
                            print(f"---m.node_cost--{m.node_cost}")
                            if m.node_cost < sys.maxsize:
                                count_next = next(tiebreaker)
                                frontier.put((m.node_cost, count_next, m))  # Include the tiebreaker in the tuple



def cost(solution):
    """
    Calculate the total cost of a solution, which is the sum of the lengths of all paths.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: The total cost as an integer.
    """
    total_cost = 0
    for path in solution.values():
        # print(f"---path--{path}")
        if path is not None:
            total_cost += len(path)  # Add the length of this agent's path to the total cost
    return total_cost


def find_first_conflict(solution, initial_positions):
    """
    Find the first conflict in the solution.

    :param solution: A dictionary mapping entity(agent/box) IDs to their respective paths (lists of actions).
    :return: A conflict object with details about the conflict, or None if no conflict is found.
    """
    # Create a dictionary to track positions of each entity at each time step
    positions = {}
    avoid_pos_list = {}
    # Find the maximum path length to know how many time steps to check
    max_path_length = max(len(path) for path in solution.values())
    print(f"---max_path_length--{max_path_length}")

    # sort agent by their path length, so that we always start from short path
    sorted_agents = sorted(solution.items(), key=lambda item: len(item[1]))

    for agent_id, path in sorted_agents:
        print(f"---agent_id--{agent_id}")
        print(f'---box is--{initial_positions[agent_id]["box_id"]}')
        print(f'---path is--{path}')

        # agent-agent conflict
        if initial_positions[agent_id]['box_id'] is None:
            current_position = initial_positions[agent_id]['agent_position']
            print(f"---current_position--{current_position}")
            print(f"---agent_id--{agent_id}")
            print(f"---path--{path}")
            time_step = 1 # Start the first step at 1
            # Loop over the max path length to avoid situation that one agent reach the goal and stop moving, but still block the other agents
            while time_step < max_path_length:
                if time_step < len(path)+1:
                    action_list = path[time_step - 1]
                    action = action_list[0]
                    print(f"---action--{action}")
                    # Calculate the resulting position of the agent after the action
                    resulting_position = Position(
                        current_position.x + action.agent_rel_pos.x,
                        current_position.y + action.agent_rel_pos.y
                    )
                    if (resulting_position, time_step) in positions:
                        # Conflict detected, return information about the conflict
                        other_agent_id = positions[(resulting_position, time_step)]
                        print(f"---Conflict--{Conflict(agent_id, other_agent_id, resulting_position, time_step)}")
                        return Conflict(agent_id, other_agent_id, resulting_position, time_step)

                 # If the agent has reached the goal, still extend its path to avoid blocking other agents
                else:
                    resulting_position = current_position
                    if (resulting_position, time_step) in positions:
                        # Conflict detected, return information about the conflict
                        other_agent_id = positions[(resulting_position, time_step)]
                        print(f"---MoveAwayConflict--{MoveAwayConflict(agent_id, resulting_agent_position, avoid_pos_list, time_step-1)}")
                        return MoveAwayConflict(agent_id, resulting_agent_position, avoid_pos_list, time_step-1)

                positions[(resulting_position, time_step)] = agent_id
                print(f"---positions--{positions}")
                # Update the current position of the agent
                current_position = resulting_position
                time_step += 1

        # agent-box conflict
        # box-box conflict
        else:
            current_agent_position = initial_positions[agent_id]['agent_position']
            box_id = initial_positions[agent_id]['box_id']
            current_box_position = initial_positions[agent_id]['box_position']
            print(f"---current_agent_position--{current_agent_position}")
            print(f"---current_box_position--{current_box_position}")
            print(f"---box_id--{box_id}")
            time_step = 1
            # Iterate for max_path_length steps
            for time_step in range(1, max_path_length + 1):
                print(f"---time_step--{time_step}")
                # Agent is within its path
                if time_step < len(path)+1:
                    action_list = path[time_step - 1]
                    action = action_list[0]
                    print(f"---action--{action}")
                    resulting_agent_position = Position(
                        current_agent_position.x + action.agent_rel_pos.x,
                        current_agent_position.y + action.agent_rel_pos.y
                    )
                    resulting_box_position = Position(
                        current_box_position.x + action.box_rel_pos.x,
                        current_box_position.y + action.box_rel_pos.y
                    )
                    print(f"---resulting_agent_position--{resulting_agent_position}")
                    print(f"---resulting_box_position--{resulting_box_position}")
                    if (resulting_agent_position, time_step) in positions:
                        other_entity_id = positions[(resulting_agent_position, time_step)]
                        print(f"---Conflict--{Conflict(agent_id, other_entity_id, resulting_agent_position, time_step)}")
                        return Conflict(agent_id, other_entity_id, resulting_agent_position, time_step)
                    elif (resulting_box_position, time_step) in positions:
                        other_entity_id = positions[(resulting_box_position, time_step)]
                        print(f"---Conflict--{Conflict(box_id, other_entity_id, resulting_box_position, time_step)}")
                        return Conflict(box_id, other_entity_id, resulting_box_position, time_step)

                # Agent has reached the goal, but still extend its path to avoid blocking other agents
                else:
                    resulting_agent_position = current_agent_position
                    resulting_box_position = current_box_position
                    if (resulting_agent_position, time_step) in positions:
                        other_entity_id = positions[(resulting_agent_position, time_step)]
                        print(f'###### positions--{positions}########')
                        avoid_pos_list = {pos: agent_id for pos, agent_id in positions.items() if pos[1] >= time_step-1}
                        print(f'---avoid_pos_list 1--{avoid_pos_list}')
                        print(f"---MoveAwayConflict 1 --{MoveAwayConflict(agent_id, resulting_agent_position, avoid_pos_list, time_step-1)}")
                        return MoveAwayConflict(agent_id, resulting_agent_position, avoid_pos_list, time_step-1)
                    # box cannot move away when it reaches the goal, so need to return normal conflict for the other agent to replan
                    elif (resulting_box_position, time_step) in positions:
                        other_entity_id = positions[(resulting_box_position, time_step)]
                        print(f"---Conflict--{Conflict(box_id, other_entity_id, resulting_box_position, time_step)}")
                        return Conflict(box_id, other_entity_id, resulting_box_position, time_step)
                    print(f"---resulting_agent_position in else--{resulting_agent_position}")
                    print(f"---resulting_box_position in else--{resulting_box_position}")

                positions[(resulting_agent_position, time_step)] = agent_id
                positions[(resulting_box_position, time_step)] = box_id
                print(f"---positions--{positions}")
                current_agent_position = resulting_agent_position
                current_box_position = resulting_box_position
                time_step += 1

    return None


def merge_plans(plans):
    """
    Merge the individual plans of the agents into a single executable plan.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: A list of joint actions that represents the executable plan.
    """
    print(f"---plans--{plans}")
    # Find the maximum length of the individual agent plans
    max_length = max(len(plan) for plan in plans.values())

    # Initialize the merged plan
    merged_plan = []

    # Iterate over each time step
    for step in range(max_length):
        joint_action = []

        # For each agent, get the action at the current step or use NoOp if the plan is shorter
        for agent_id in sorted(plans.keys()):  # Sort the agent IDs to maintain order
            plan = plans[agent_id]
            if step < len(plan):
                action = plan[step][0]  # Each action is a list, take the first element
            else:
                # Use a specific NoOp action
                action = Action.NoOp
            joint_action.append((agent_id, action))  # Include agent_id for sorting

        # Sort joint_action by agent_id and extract the actions in order
        joint_action.sort(key=lambda x: x[0])
        joint_action = [action for agent_id, action in joint_action]

        merged_plan.append(joint_action)
        print(f'merged_plan--{merged_plan}')
    return merged_plan


def solve_moveaway_conflict(node, conflict):
    agent_id = conflict.ai
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
                print(f"---agent_destination--{agent_destination}")
                print(f"---action--{action}")
                insert_action = [action]
    # print(f"---insert_action--{insert_action}")
    # print(f"---node.solution[agent_id]--{node.solution[agent_id]}")

    # Fill the solution with NoOp until the time step
    while len(node.solution[agent_id]) < time_step-1:
        node.solution[agent_id].append([Action.NoOp])
    # Insert the new action at the time step
    node.solution[agent_id].insert(time_step-1, insert_action)
    # Return the new solution for the agent that needs to move away.
    return node.solution[agent_id]
