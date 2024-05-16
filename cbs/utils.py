"""Common functions used by other modules."""
import sys

from domain.action import Action, ActionType
from htn.htn_resolver import HTNResolver

def find_meta_agent(agent_id_i, agent_id_j, agent_i_pos_list, box_i_pos_list, agent_j_pos_list, box_j_pos_list):
    # Initialize meta agent ID
    meta_agent_id = None

    # Check if agent_i or box_i is blocked by agent_j or box_j at any position
    for pos_i in agent_i_pos_list + box_i_pos_list:
        if pos_i in agent_j_pos_list or pos_i in box_j_pos_list:
            meta_agent_id = agent_id_j
            break

    # If agent_i is not blocked, check if agent_j or box_j is blocked by agent_i or box_i
    if meta_agent_id is None:
        for pos_j in agent_j_pos_list + box_j_pos_list:
            if pos_j in agent_i_pos_list or pos_j in box_i_pos_list:
                meta_agent_id = agent_id_i
                break

    # If neither agent is blocked by the other, choose the first one as the meta agent
    if meta_agent_id is None:
        meta_agent_id = agent_id_i

    # Return the meta agent ID
    return meta_agent_id


def get_actual_agent_id(initial_positions, entity_id):
    # If the entity_id is a box, return its corresponding agent_id
    for agent_id, pos in initial_positions.items():
        if pos['box_id'] == entity_id:
            return agent_id
    # If the entity_id is already an agent_id, return it as is
    return entity_id


def get_pos_list(agent_id, initial_positions, initial_solutions):
    agent_pos_list = []
    box_pos_list = []
    box_flag = False
    agent_current_pos = initial_positions[agent_id]['agent_position']
    agent_pos_list.append((agent_current_pos, 0))
    if initial_positions[agent_id]['box_id'] is not None:
        box_flag = True
        box_current_pos = initial_positions[agent_id]['box_position']
        box_pos_list.append((box_current_pos, 0))
    for step in range(len(initial_solutions[agent_id])):
        action = initial_solutions[agent_id][step]
        agent_destination = agent_current_pos + action.agent_rel_pos
        agent_pos_list.append((agent_destination, step+1))
        agent_current_pos = agent_destination
        if box_flag:
            box_destination = box_current_pos + action.box_rel_pos
            box_pos_list.append((box_destination, step+1))
            box_current_pos = box_destination
    return agent_pos_list, box_pos_list


def get_resulting_positions_of_plan(non_meta_agent_id, initial_positions, move_out_plan):
    agent_pos_list, box_pos_list = get_pos_list(non_meta_agent_id, initial_positions, move_out_plan)
    return agent_pos_list[-1][0], box_pos_list[-1][0]

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

    for agent_uid, task in round.items():
        if(HTNResolver.completed_tasks.get(agent_uid) is None):
            HTNResolver.completed_tasks[agent_uid] = []
        HTNResolver.completed_tasks[agent_uid].append(task)
    # print(f"round--{round}", file=sys.stderr)
    return merged_plan


def opposite_action(action):
    if action == Action.MoveN:
        return Action.MoveS
    elif action == Action.MoveS:
        return Action.MoveN
    elif action == Action.MoveE:
        return Action.MoveW
    elif action == Action.MoveW:
        return Action.MoveE
    else:
        return Action.NoOp
