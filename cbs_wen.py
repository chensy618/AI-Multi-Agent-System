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
    initial_positions = []

    # Get initial positions
    for problem in problem_list:
        # Iterate over agents, assuming that not all agents may have a corresponding box
        for agent in problem.agents:
            # Find the corresponding box if it exists
            # TODO: Can be optimized by using a dictionary to map agent to boxes, instead of using color again here
            box = next((b for b in problem.boxes if b.color == agent.color), None)
            # Create a tuple with the agent and box information, as tuple can handle the order of data very well, and is immutable.
            # If there is no corresponding box, use None for the box ID and position
            initial_position = (agent.id, agent.pos, box.id if box else None, box.pos if box else None)
            initial_positions.append(initial_position)
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
        else:
            print(f"---conflict--{conflict}")
        for problem in problem_list:
            for agent in problem.agents:
                print(f"----------------------------agent.id--{agent.id}")
                if agent.id in [conflict.ai, conflict.aj]:
                    m = node.copy()
                    m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t))
                    m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+1))
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
        print(f"---path--{path}")
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

    for agent_id, path in solution.items():
        print(f"---agent_id--{agent_id}")
        print(f'---box is--{initial_positions[agent_id][2]}')

        # agent-agent conflict
        if initial_positions[agent_id][2] is None:
            current_position = initial_positions[agent_id][1]
            print(f"---current_position--{current_position}")
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
        # agent-box conflict
        # box-box conflict
        else:
            current_agent_position = initial_positions[agent_id][1]
            box_id = initial_positions[agent_id][2]
            current_box_position = initial_positions[agent_id][3]
            print(f"---current_agent_position--{current_agent_position}")
            print(f"---current_box_position--{current_box_position}")
            print(f"---box_id--{box_id}")
            time_step = 1
            for action_list in path:
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
                    return Conflict(agent_id, other_entity_id, resulting_agent_position, time_step)
                elif (resulting_box_position, time_step) in positions:
                    other_entity_id = positions[(resulting_box_position, time_step)]
                    return Conflict(box_id, other_entity_id, resulting_box_position, time_step)
                positions[(resulting_agent_position, time_step)] = agent_id
                positions[(resulting_box_position, time_step)] = box_id
                print(f"---positions--{positions}")
                time_step += 1

    return None


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
