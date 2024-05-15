"""Handle agent communication."""
import sys

from astar import astar
from cbs.utils import find_meta_agent, get_actual_agent_id, get_pos_list, get_resulting_positions_of_plan, opposite_action
from domain.action import Action, ActionType
from domain.conflict import FollowConflict

def ask_blocked_agent_help(agent_id, blocked_agent_id, agent_current_pos, time_step, node, avoid_pos_list, walls):
    """ The agent_id needs to move away so that blocked_agent_id can move.
    However the agent_id doesn't have avaialble action to move away,
    so this function is called to ask the blocked_agent_id to move away so that agent_id can move,
    and make the path available for the blocked_agent_id."""
    print(f'original solution for agent {agent_id}: {node.solution[agent_id]}',file=sys.stderr)
    print(f'original solution for blocked agent {blocked_agent_id}: {node.solution[blocked_agent_id]}',file=sys.stderr)
    # Get the current position of the blocked agent
    for (position, t), info in avoid_pos_list.items():
        if info['id'] == blocked_agent_id and t == time_step:
            blocked_agent_current_pos = position
    print(f'Agent {blocked_agent_id} is blocked by agent {agent_id} at time step {time_step} at position {agent_current_pos}',file=sys.stderr)
    # Used for counting the number of actions to the agent
    count = 0
    # loop actions to see the blocked agent's action at the time step
    for action in Action:
        if action.type == ActionType.Move:
            blocked_agent_destination = blocked_agent_current_pos + action.agent_rel_pos
            # Check if agent_destination is within the bounds of the walls matrix
            if 0 <= blocked_agent_destination.x < len(walls[0]) and 0 <= blocked_agent_destination.y < len(walls):
                # Check if agent_destination is a wall or in avoid_positions
                is_destination_occupied = walls[blocked_agent_destination.y][blocked_agent_destination.x] or \
                                            any(pos == blocked_agent_destination for pos, _ in avoid_pos_list.keys())
                if not is_destination_occupied:
                    # add the position and time step to the avoid_pos_list
                    avoid_pos_list[blocked_agent_destination, time_step+1] = {'id': blocked_agent_id}
                    # filter out the blocked agent's current position from the avoid_pos_list so that the agent can move into it later
                    new_avoid_pos_list = {
                        pos_t: info for pos_t, info in avoid_pos_list.items()
                        if pos_t[1] != time_step or info['id'] != blocked_agent_id
                    }
                    print(f'---new_avoid_pos_list: {new_avoid_pos_list}---',file=sys.stderr)
                    insert_blocked_agent_action = action
                    print(f'---insert_blocked_agent_action: {insert_blocked_agent_action}---',file=sys.stderr)
                    # Insert the new action at the time step
                    node.solution[blocked_agent_id].insert(time_step, insert_blocked_agent_action)
                    print(f'node.solution[blocked_agent_id]: {node.solution[blocked_agent_id]}',file=sys.stderr)
                    # Update the solution for the agent
                    # Firstly wait for the blocked agent to move
                    # Insert NoOp for the agent if the agent's solution is shorter than the current time step
                    while len(node.solution[agent_id]) < time_step:
                        node.solution[agent_id].append(Action.NoOp)
                    node.solution[agent_id].insert(time_step, Action.NoOp)
                    print(f'node.solution[agent_id]: {node.solution[agent_id]}',file=sys.stderr)
                    while count < 3:
                        # Then find the next 3 move for the agent
                        for action_agent in Action:
                            if action_agent.type == ActionType.Move:
                                agent_destination = agent_current_pos + action_agent.agent_rel_pos
                                # Check if agent_destination is within the bounds of the walls matrix
                                if 0 <= agent_destination.x < len(walls[0]) and 0 <= agent_destination.y < len(walls):
                                    # Check if agent_destination is a wall or in avoid_positions
                                    is_destination_occupied = walls[agent_destination.y][agent_destination.x] or \
                                                                any(pos == agent_destination for pos, _ in new_avoid_pos_list.keys())
                                    print(f'---is_destination_occupied: {is_destination_occupied}---',file=sys.stderr)
                                    if not is_destination_occupied:
                                        # add the position and time step to the new_avoid_pos_list
                                        new_avoid_pos_list[agent_destination, time_step+count+2] = {'id': agent_id}
                                        insert_agent_action = action_agent
                                        print(f'---insert_agent_action: {insert_agent_action}---',file=sys.stderr)
                                        print(f'---count: {count}---',file=sys.stderr)
                                        # Insert the new action at the time step
                                        node.solution[agent_id].insert(time_step+count+1, insert_agent_action)
                                        # The blocked agent id needs to wait for the agent to move
                                        node.solution[blocked_agent_id].insert(time_step+count+1, Action.NoOp)
                                        print(f'node.solution[agent_id]: {node.solution[agent_id]}',file=sys.stderr)
                                        print(f'node.solution[blocked_agent_id]: {node.solution[blocked_agent_id]}',file=sys.stderr)
                                        # Update the current position of the agent
                                        agent_current_pos = agent_destination
                        count += 1
                    # Insert the opposite action to the blocked agent so that it can return
                    node.solution[blocked_agent_id].insert(time_step+count+1, opposite_action(insert_blocked_agent_action))
                    print(f'final node.solution[block ed_agent_id]: {node.solution[blocked_agent_id]}',file=sys.stderr)
                    # break the loop as long as 1 available action is found
                    break

    return node


def meta_agent_block_communication(node, initial_solutions, initial_positions, conflict, current_state, round):
    # if isinstance(conflict, FollowConflict):
    #     print(f"The meta agent communication does not support follow conflict handling", file=sys.stderr)
    #     return node
    # Decide which one is the new meta agent, and which one is the agent that needs to take actions
    # Get the agent and box ids
    entity_i = conflict.ai
    entity_j = conflict.aj
    agent_id_i = get_actual_agent_id(initial_positions, entity_i)
    if initial_positions[agent_id_i]['box_id'] is not None:
        box_id_i = initial_positions[agent_id_i]['box_id']
    agent_id_j = get_actual_agent_id(initial_positions, entity_j)
    agent_i_pos_list, box_i_pos_list = get_pos_list(agent_id_i, initial_positions, initial_solutions)
    agent_j_pos_list, box_j_pos_list = get_pos_list(agent_id_j, initial_positions, initial_solutions)

    # Find the meta agent
    meta_agent_id = find_meta_agent(
        agent_id_i, agent_id_j,
        agent_i_pos_list, box_i_pos_list,
        agent_j_pos_list, box_j_pos_list
    )

    # Get meta box from the meta agent
    meta_box_id = initial_positions[meta_agent_id]['box_id']
    meta_agent_pos_list = agent_i_pos_list if meta_agent_id == agent_id_i else agent_j_pos_list
    meta_box_pos_list = box_i_pos_list if box_id_i == initial_positions[meta_agent_id]['box_id'] else box_j_pos_list
    # print(f'meta_agent_pos_list: {meta_agent_pos_list}',file=sys.stderr)
    # print(f'meta_box_pos_list: {meta_box_pos_list}',file=sys.stderr)
    # print(f'-----------------meta_agent_id: {meta_agent_id}',file=sys.stderr)
    # print(f'-----------------meta_box_id: {meta_box_id}',file=sys.stderr)
    non_meta_agent_id = agent_id_i if meta_agent_id == agent_id_j else agent_id_j
    non_meta_box_id = initial_positions[non_meta_agent_id]['box_id']
    # print(f'-----------------non_meta_agent_id: {non_meta_agent_id}',file=sys.stderr)
    # print(f'-----------------non_meta_box_id: {non_meta_box_id}',file=sys.stderr)

    # Store the path of the meta agent/box in avoid_pos_list, and move the other agent/box out of the way
    # Add meta agent/box initial position to walls
    walls = current_state.walls
    walls[initial_positions[meta_agent_id]['agent_position'].y][initial_positions[meta_agent_id]['agent_position'].x] = True
    walls[initial_positions[meta_agent_id]['box_position'].y][initial_positions[meta_agent_id]['box_position'].x] = True
    # print(f'walls: {walls}',file=sys.stderr)
    # avoid_pos_list is the list of positions that the agent/box needs to move out from
    # use avoid_pos_list to find the temporary goal for the non-meta agent/box
    avoid_pos_list = meta_agent_pos_list + meta_box_pos_list
    # print(f'avoid_pos_list: {avoid_pos_list}',file=sys.stderr)
    temp_goal = find_temp_goal(avoid_pos_list, walls)
    for goal in current_state.goals:
        # print(f'goal: {goal}',file=sys.stderr)
        # print(f'goal.value: {goal.value}',file=sys.stderr)
        if goal.value == non_meta_box_id:
            real_goal = goal.pos
            break
    # print(f'current state goals: {current_state.goals}',file=sys.stderr)
    # print(f'temp_goal: {temp_goal}',file=sys.stderr)
    # print(f'real_goal: {real_goal}',file=sys.stderr)

    # Update the current state goal map and goal position to use the temp goal
    goal_uid = round[non_meta_agent_id].goal_uid
    # print(f'round is {round}',file=sys.stderr)
    # print(f'goal_uid: {goal_uid}',file=sys.stderr)
    current_state.goals[goal_uid].pos = temp_goal
    current_state.goal_map[goal_uid][temp_goal.y][temp_goal.x] = 0
    current_state.goal_map[goal_uid][real_goal.y][real_goal.x] = 1
    # print(f'current state goal map is {current_state.goal_map[goal_uid]}',file=sys.stderr)
    # print(f'current state non meta agent goal is {current_state.goals[goal_uid]}',file=sys.stderr)
    # Invoke Astar to find the path for the non meta agent/box to move out of the way
    relaxed_state = current_state.from_agent_perspective(non_meta_agent_id)
    try:
        move_out_plan = astar(relaxed_state, round[non_meta_agent_id])
        # print(f'-------round[non_meta_agent_id]: {round[non_meta_agent_id]}-------',file=sys.stderr)
        print(f'==================================move_out_plan: {move_out_plan}',file=sys.stderr)
    except RuntimeError as e:
        print(f"The meta agent communication cannot solve the move_out_plan: {e}", file=sys.stderr)
        return node
    finally:
        # To avoid that the meta agent block the way of the agent/box that needs to move
        # Change the goal back to the real goal after the agent/box moved out of the way
        current_state.goals[goal_uid].pos = real_goal
        current_state.goal_map[goal_uid][temp_goal.y][temp_goal.x] = 1
        current_state.goal_map[goal_uid][real_goal.y][real_goal.x] = 0
        # print(f'current state goal map is {current_state.goal_map[goal_uid]}',file=sys.stderr)
        # print(f'current state non meta agent goal is {current_state.goals[goal_uid]}',file=sys.stderr)
        # print(f'current state agents: {current_state.agents}',file=sys.stderr)

    # update the initial positions of the non-meta agent/box to the moving out plan resulting positions
    # replan for the non-meta agent/box
    resulting_position_agent, resulting_position_box = get_resulting_positions_of_plan(non_meta_agent_id, initial_positions, {non_meta_agent_id: move_out_plan})
    # print(f'-------------------resulting_position_agent: {resulting_position_agent}',file=sys.stderr)
    # print(f'-------------------resulting_position_box: {resulting_position_box}',file=sys.stderr)
    # Find the non meta agent value and update the position
    for agent in current_state.agents:
        if agent.value == non_meta_agent_id:
            agent.pos = resulting_position_agent
            break
    # print(f'After change current state agent: {current_state.agents}',file=sys.stderr)
    # Find the non meta box value and update the position
    if initial_positions[non_meta_agent_id]['box_id'] is not None:
        box_uid = round[non_meta_agent_id].box_uid
        for uid, box in current_state.boxes.items():
            if box.value == non_meta_box_id and uid == box_uid:
                # print('box position has been reset',file=sys.stderr)
                current_state.boxes[uid].pos = resulting_position_box
                # print(f'After change current state box: {current_state.boxes[uid].pos}',file=sys.stderr)
                break

    relaxed_state = current_state.from_agent_perspective(non_meta_agent_id)
    try:
        non_meta_agent_rest_plan = astar(relaxed_state, round[non_meta_agent_id])
        print(f'==================================non_meta_agent_rest_plan: {non_meta_agent_rest_plan}',file=sys.stderr)
    except RuntimeError as e:
        print(f"The meta agent communication cannot solve the non_meta_agent_rest_plan: {e}", file=sys.stderr)
        return node
    finally:
        # Update the initial positions of the non-meta agent/box to the real initial positions
        for agent in current_state.agents:
            if agent.value == non_meta_agent_id:
                agent.pos = initial_positions[non_meta_agent_id]['agent_position']
                break
        if initial_positions[non_meta_agent_id]['box_id'] is not None:
            for uid, box in current_state.boxes.items():
                if box.value == non_meta_box_id and box.uid == box_uid:
                    current_state.boxes[uid].pos = initial_positions[non_meta_agent_id]['box_position']
                    break
        # print(f'================================The current state agent has been changed back to the initial position: {current_state.agents}',file=sys.stderr)
        # print(f'================================The current state box has been changed back to the initial position: {current_state.boxes}',file=sys.stderr)
        # print(f'================================The current state goal map has been changed back to the initial position: {current_state.goal_map}',file=sys.stderr)
        # print(f'================================The current state goals has been changed back to the initial position: {current_state.goals}',file=sys.stderr)
        # Remove the temp walls
        walls[initial_positions[meta_agent_id]['agent_position'].y][initial_positions[meta_agent_id]['agent_position'].x] = False
        walls[initial_positions[meta_agent_id]['box_position'].y][initial_positions[meta_agent_id]['box_position'].x] = False

    # Wait while the meta agent/box is moving towards the goal
    non_meta_agent_noop_plan = [Action.NoOp for _ in range(len(initial_solutions[meta_agent_id]))]
    # Insert the available actions to the agent/box that needs to move
    node.solution[non_meta_agent_id] = move_out_plan + non_meta_agent_noop_plan + non_meta_agent_rest_plan
    # print(f'node.solution[non_meta_agent_id]: {node.solution[non_meta_agent_id]}',file=sys.stderr)
    # Insert the NoOp action to the meta agent/box from the beginning, until the agent/box moved out of the way
    for _ in range(len(move_out_plan)):
        node.solution[meta_agent_id].insert(0, Action.NoOp)
    # print(f'node.solution[meta_agent_id]: {node.solution[meta_agent_id]}',file=sys.stderr)
    # print(f'---node.solution: {node.solution}---',file=sys.stderr)

    # Once the agent/box moved out, insert the NoOp action to the agent/box that needs to move
    return node


def find_temp_goal(avoid_pos_list, walls):
    start_time_step = max([t for _, t in avoid_pos_list])
    pos_list = []
    print(f'start_time_step: {start_time_step}',file=sys.stderr)
    for time_step in range(start_time_step, -1, -1):
        # print(f'----------t: {time_step}-----------',file=sys.stderr)
        for pos, t in avoid_pos_list:
            if t == time_step:
                print(pos)
                pos_list.append(pos)
        # print(f'----------pos_list: {pos_list}--------',file=sys.stderr)
        for pos in pos_list:
            # print(f'pos: {pos}',file=sys.stderr)
            for action in Action:
                if action.type == ActionType.Move:
                    neighbor_pos = pos + action.agent_rel_pos
                    if neighbor_pos is not None and 0 <= neighbor_pos.x < len(walls[0]) and 0 <= neighbor_pos.y < len(walls):
                        is_neighbor_occupied = walls[neighbor_pos.y][neighbor_pos.x] or \
                                                any(pos == neighbor_pos for pos in pos_list) or \
                                                any(pos == neighbor_pos for pos, _ in avoid_pos_list)
                        if not is_neighbor_occupied:
                            # TODO: move the temp goal 1 more step away from the avoid_pos_list
                            return neighbor_pos
    return None

