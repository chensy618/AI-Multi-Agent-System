import itertools
import sys

from astar import astar
from cbs.node import Node

from domain.action import Action
from domain.conflict import Conflict
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
    # initial_box_positions = {}
    for problem in problem_list:
        for agent in problem.agents:
            print(f"---agent--{agent.id}")
            print(f"---problem--{problem}")
            initial_positions[agent.id] = agent.pos
            root.solution[agent.id] = astar(problem)
        ############## Related boxes position code has been commented out
        # # but actually needed when considering more types of conflicts ##############
        # # Store initial positions of boxes
        # for box in problem.boxes:
        #     print(f"---box--{box.id}")
        #     initial_box_positions[box.id] = box.pos
    print(f"---initial_positions--{initial_positions}")
    # print(f"---initial_box_positions--{initial_box_positions}")
    root.cost = cost(root.solution)
    frontier = PriorityQueue()
    count_next = next(tiebreaker)
    frontier.put((root.cost, count_next, root))
    while not frontier.empty():
        node_cost, tiebreaker_value, node = frontier.get()
        conflict = find_first_conflict(node.solution, initial_positions)
        # conflict = find_first_conflict(node.solution, initial_positions, initial_box_positions)
        if conflict is None:
            print('I am here')
            executable_plan = merge_plans(node.solution)
            return executable_plan
        else:
            print(f"---conflict--{conflict}")
        for problem in problem_list:
            for agent in problem.agents:
                print(f"---agent--{agent.id}")
                if agent.id in [conflict.ai, conflict.aj]:
                    m = node.copy()
                    print(f"---m--{m}")
                    print(Position(conflict.pos.x, conflict.pos.y))
                    m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t))
                    print(f"---m.constraints--{m.constraints}")
                    m.solution[agent.id] = space_time_a_star(problem, m.constraints)
                    print(f"---m.solution--{m.solution[agent.id]}")
                    m.node_cost = cost(m.solution)
                    print(f"---m.node_cost--{m.node_cost}")

                    # When putting a node into the priority queue, include the tiebreaker
                    # To be able to solve the issue when the costs are equal
                    if m.node_cost < sys.maxsize:
                        count_next = next(tiebreaker)  # Get the next value from the tiebreaker
                        frontier.put((m.node_cost, count_next, m))  # Include the tiebreaker in the tuple


def cost(solution):
    """
    Calculate the total cost of a solution, which is the sum of the lengths of all paths.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: The total cost as an integer.
    """
    total_cost = 0
    for path in solution.values():
        print(f"---path--{path}")
        if path is not None:
            total_cost += len(path)  # Add the length of this agent's path to the total cost
    return total_cost


#################### Currently the level is an agent-box conflict. so need to adjust the code####################
def find_first_conflict(solution, initial_positions):
    """
    Find the first conflict in the solution.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: A conflict object with details about the conflict, or None if no conflict is found.
    """
    # Create a dictionary to track positions of each agent at each time step
    positions = {}
    for agent_id, path in solution.items():
        # Get the initial position of the agent
        current_position = initial_positions[agent_id]
        # print(f"---current_position--{current_position}")
        # print(f"---agent_id--{agent_id}")
        # print(f"---path--{path}")
        time_step = 1 # Start the first step at 1
        for action_list in path:
            action = action_list[0]  # Assuming each action is wrapped in a list
            # print(f"---action--{action}")
            # Calculate the resulting position of the agent after the action
            resulting_position = Position(
                current_position.x + action.agent_rel_pos.x,
                current_position.y + action.agent_rel_pos.y
            )
            if (resulting_position, time_step) in positions:
                # Conflict detected, return information about the conflict
                other_agent_id = positions[(resulting_position, time_step)]
                # print(f"---agent_id--{agent_id}")
                # print(f"---other_agent_id--{other_agent_id}")
                # print(f"---Conflict--{Conflict(agent_id, other_agent_id, resulting_position, time_step)}")
                return Conflict(agent_id, other_agent_id, resulting_position, time_step)
            positions[(resulting_position, time_step)] = agent_id
            # print(f"---positions--{positions}")
            # Update the current position of the agent
            current_position = resulting_position
            time_step += 1
    return None


# ################### Need to consider more types of conflicts as well####################
# # Agent-Agent Conflict
# # Agent-Box Conflict
# # Box-Box Conflict
# # following conflict
# ################### Below code need big adjustment####################
# def find_first_conflict(solution, initial_positions, initial_box_positions):
#     """
#     Find the first conflict in the solution.

#     :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
#     :param initial_positions: A dictionary mapping agent IDs to their initial positions.
#     :param initial_box_positions: A dictionary mapping box IDs to their initial positions.
#     :return: A conflict object with details about the conflict, or None if no conflict is found.
#     """
#     # Create a dictionary to track positions of each agent and box at each time step
#     positions = {}
#     box_positions = {pos: box_id for box_id, pos in initial_box_positions.items()}
#     for agent_id, path in solution.items():
#         # Get the initial position of the agent
#         current_position = initial_positions[agent_id]
#         time_step = 1  # Start the first step at 1
#         for action_list in path:
#             action = action_list[0]  # Assuming each action is wrapped in a list
#             # Calculate the resulting position of the agent after the action
#             resulting_position = Position(
#                 current_position.x + action.agent_rel_pos.x,
#                 current_position.y + action.agent_rel_pos.y
#             )
#             # Check for agent-agent conflicts
#             if (resulting_position, time_step) in positions:
#                 # Conflict detected, return information about the conflict
#                 other_agent_id = positions[(resulting_position, time_step)]
#                 return Conflict(agent_id, other_agent_id, resulting_position, time_step)
#             # Check for agent-box conflicts
#             if action.type in [ActionType.Push, ActionType.Pull]:
#                 # Calculate the new position of the box after the action
#                 box_new_position = Position(
#                     current_position.x + action.box_rel_pos.x,
#                     current_position.y + action.box_rel_pos.y
#                 )
#                 if box_new_position in box_positions and box_positions[box_new_position] != agent_id:
#                     # Conflict detected between agent and box
#                     return Conflict(agent_id, box_positions[box_new_position], box_new_position, time_step)
#                 # Update the box position
#                 if action.type == ActionType.Push:
#                     box_positions.pop(current_position, None)  # Remove the old box position
#                     box_positions[box_new_position] = agent_id  # Add the new box position
#                 elif action.type == ActionType.Pull:
#                     box_positions.pop(resulting_position, None)  # Remove the old box position
#                     box_positions[current_position] = agent_id  # Add the new box position

#             positions[(resulting_position, time_step)] = agent_id
#             # Update the current position of the agent
#             current_position = resulting_position
#             time_step += 1
#     return None


def merge_plans(plans):
    """
    Merge the individual plans of the agents into a single executable plan.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: A list of joint actions that represents the executable plan.
    """
    # Find the maximum length of the individual agent plans
    max_length = max(len(plan) for plan in plans.values())

    # Initialize the merged plan
    merged_plan = []

    # Iterate over each time step
    for step in range(max_length):
        joint_action = []

        # For each agent, get the action at the current step or use NoOp if the plan is shorter
        for agent_id, plan in plans.items():
            if step < len(plan):
                action = plan[step][0]  # Each action is a list, take the first element
            else:
                # Assuming NoOp is represented as None or a specific NoOp action
                action = Action.NoOp
            joint_action.append(action)

        # Append the joint action to the merged plan
        merged_plan.append(joint_action)
    print(f"---merged_plan--{merged_plan}")
    return merged_plan
