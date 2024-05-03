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
from state import State

# Create a counter that will serve as a tiebreaker
tiebreaker = itertools.count()

def conflict_based_search(current_state: State, round):
    root = Node()
    root.constraints = []

    for agent in current_state.agents:
        relaxed_state = current_state.from_agent_perspective(agent.uid)
        something = astar(relaxed_state, round)
        print('something', something, file=sys.stderr)
        root.solution[agent.uid] = something
        
    root.cost = cost(root.solution)
    frontier = PriorityQueue()
    count_next = next(tiebreaker)
    frontier.put((root.cost, count_next, root))
    while not frontier.empty():
        node_cost, tiebreaker_value, node = frontier.get()
        # based on the conflict type, we only support to call the corresponding conflict solve method
        conflict = find_first_conflict_box_box(node.solution, round, initial_box_positions)
        #conflict = find_first_conflict_agt_agt(node.solution, initial_agent_positions)
        #conflict = find_first_conflict_agt_box(node.solution, problem_list, initial_agent_positions, initial_box_positions)
        if conflict is None:
            print('I am here, no conflict found.', file=sys.stderr)
            executable_plan = merge_plans(node.solution)
            return executable_plan
        else:
            print(f"---conflict--{conflict}", file=sys.stderr)
        for problem in round:
            # this is the box-box conflict solve method : start from here
            for box, agent in zip(problem.boxes, problem.agents):
                if box.id in [conflict.ai, conflict.aj]:
                    m = node.copy()
                    print(f"---m--{m}")
                    print(Position(conflict.pos.x, conflict.pos.y))
                    m.constraints.append(Constraint(box.id, Position(conflict.pos.x, conflict.pos.y), conflict.t))
                    m.constraints.append(Constraint(box.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+1))
                    m.constraints.append(Constraint(box.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+2))
                    print(f"---m.constraints--{m.constraints}", file=sys.stderr)
                    m.solution[agent.id] = space_time_a_star(problem, m.constraints, round)
                    print(f"---m.solution--{m.solution[agent.id]}", file=sys.stderr)
                    m.node_cost = cost(m.solution)
                    print(f"---m.node_cost--{m.node_cost}", file=sys.stderr)

                    # When putting a node into the priority queue, include the tiebreaker
                    # To be able to solve the issue when the costs are equal
                    if m.node_cost < sys.maxsize:
                        count_next = next(tiebreaker)
                        frontier.put((m.node_cost, count_next, m))  # Include the tiebreaker in the tuple
            # ----------end of box-box conflict solve -----------------------------------------------------------------
            
            # this is the agent-agent conflict solve method : start from here
            # for agent in problem.agents:
            #     print(f"---agent--{agent.id}")
            #     if agent.id in [conflict.ai, conflict.aj]:
            #         m = node.copy()
            #         print(f"---m--{m}")
            #         print(Position(conflict.pos.x, conflict.pos.y))
            #         m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t))
            #         m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+1))
            #         print(f"---m.constraints--{m.constraints}")
            #         m.solution[agent.id] = space_time_a_star(problem, m.constraints)
            #         print(f"---m.solution--{m.solution[agent.id]}")
            #         m.node_cost = cost(m.solution)
            #         print(f"---m.node_cost--{m.node_cost}")

            #         # When putting a node into the priority queue, include the tiebreaker
            #         # To be able to solve the issue when the costs are equal
            #         if m.node_cost < sys.maxsize:
            #             count_next = next(tiebreaker)  # Get the next value from the tiebreaker
            #             frontier.put((m.node_cost, count_next, m))  # Include the tiebreaker in the tuple
            # ----------end of agent-agent conflict solve ------------------------------------------------------------------

            #this is the agent-agent conflict solve method : start from here
            # for agent in problem.agents:
            #     print(f"---agent--{agent.id}")
            #     if agent.id in [conflict.ai, conflict.aj]:
            #         m = node.copy()
            #         print(f"---m--{m}")
            #         print(Position(conflict.pos.x, conflict.pos.y))
            #         m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t))
            #         m.constraints.append(Constraint(agent.id, Position(conflict.pos.x, conflict.pos.y), conflict.t+1))
            #         print(f"---m.constraints--{m.constraints}")
            #         m.solution[agent.id] = space_time_a_star(problem, m.constraints)
            #         print(f"---m.solution--{m.solution[agent.id]}")
            #         m.node_cost = cost(m.solution)
            #         print(f"---m.node_cost--{m.node_cost}")

            #         # When putting a node into the priority queue, include the tiebreaker
            #         # To be able to solve the issue when the costs are equal
            #         if m.node_cost < sys.maxsize:
            #             count_next = next(tiebreaker)  # Get the next value from the tiebreaker
            #             frontier.put((m.node_cost, count_next, m))  # Include the tiebreaker in the tuple
            #----------end of agent-agent conflict solve ------------------------------------------------------------------
def cost(solution):
    """
    Calculate the total cost of a solution, which is the sum of the lengths of all paths.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: The total cost as an integer.
    """
    total_cost = 0
    for path in solution.values():
        print(f"---path--{path}", file=sys.stderr)
        if path is not None:
            total_cost += len(path)  # Add the length of this agent's path to the total cost
    return total_cost


def find_first_conflict_agt_agt(solution, initial_positions):
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



def find_first_conflict_box_box(solution, problem_list, initial_box_positions):
    """
    Find the first conflict in the solution.

    :param solution: A dictionary mapping agent IDs to their respective paths (lists of actions).
    :return: A conflict object with details about the conflict, or None if no conflict is found.
    """
    # Create a dictionary to track positions of each agent at each time step
    positions = {}
    for box_id, path in solution.items():
        print(f"solution.items()--{solution.items()}", file=sys.stderr)
        # we need to optimize how to convert the id in the solution.items()
        # problem0 = problem_list[0].agents[0].id
        # problem1 = problem_list[1]
        # print(f"---problem--{problem0}")
        # print(f"---problem--{problem1}")
        box_id = problem_list[box_id].boxes[0].id
        print(f"---box_id--{box_id}", file=sys.stderr)
        # Get the initial position of box
        current_position = initial_box_positions[box_id]
        print(f"---current_position--{current_position}", file=sys.stderr)
        time_step = 1  # Start the first step at 1
        for action_list in path:
            action = action_list[0]
            # Calculate the resulting position of the box after the action
            resulting_position = Position(
                current_position.x + action.box_rel_pos.x,
                current_position.y + action.box_rel_pos.y
            )
            if (resulting_position, time_step) in positions:
                # Conflict detected, return information about the conflict
                other_agent_id = positions[(resulting_position, time_step)]
                print(f"Conflict(box_id={box_id}, other_agent_id={other_agent_id}, pos={resulting_position}, t={time_step})", file=sys.stderr)
                return Conflict(box_id, other_agent_id, resulting_position, time_step)
            positions[(resulting_position, time_step)] = box_id
            # Update the current position of the box
            current_position = resulting_position
            time_step += 1
    return None

def find_first_conflict_agt_box(solution, problem_list, initial_agent_positions, initial_box_positions):
    """
    Find the first conflict in the solution, checking for conflicts between agents and boxes.

    :param solution: Dictionary mapping IDs to their respective paths (lists of actions).
    :param problem_list: List or dictionary containing problem data for each agent, including box info.
    :param initial_agent_positions: Dictionary with initial positions of agents.
    :param initial_box_positions: Dictionary with initial positions of boxes.
    :return: A conflict object with details about the conflict, or None if no conflict is found.
    """
    # Dictionary to track positions of agents and boxes at each time step
    positions = {}

    # Loop through each agent and process their paths
    for agent_id, path in solution.items():
        # Assuming each agent has at least one box associated with it in problem_list
        box_id = problem_list[agent_id].boxes[0].id
        current_agent_position = initial_agent_positions[agent_id]
        current_box_position = initial_box_positions[box_id]
        time_step = 1  # Start the first step at 1

        for action_list in path:
            action = action_list[0]  # Assuming each action is wrapped in a list

            # Calculate new position for the agent
            new_agent_position = Position(
                current_agent_position.x + action.agent_rel_pos.x,
                current_agent_position.y + action.agent_rel_pos.y
            )

            # calculate new position for the box
            new_box_position = Position(
                current_box_position.x + action.box_rel_pos.x,
                current_box_position.y + action.box_rel_pos.y
            )
            
            
            # TODO: Implement conflict detection between agents and boxes
            # Check for conflicts between the agent's new position and all box positions
            # for other_box_id, pos in positions.get(('box', time_step), {}).items():
            #     if new_agent_position == pos:
            #         return Conflict(agent_id, other_box_id, new_agent_position, time_step)


            # Check for conflicts between the box's new position and all agent positions
            # for other_agent_id, pos in positions.get(('agent', time_step), {}).items():
            #     if new_box_position == pos:
            #         return Conflict(other_agent_id, box_id, new_box_position, time_step)

            # # Update positions in the dictionary
            # positions.setdefault(('agent', time_step), {})[agent_id] = new_agent_position
            # positions.setdefault(('box', time_step), {})[box_id] = new_box_position

            # Move to next step
            current_agent_position = new_agent_position
            current_box_position = new_box_position
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
    print(f"---merged_plan--{merged_plan}", file=sys.stderr)
    return merged_plan
