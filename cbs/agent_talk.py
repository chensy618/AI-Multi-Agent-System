"""Handle agent communication."""

from domain.action import Action, ActionType


def ask_blocked_agent_help(agent_id, blocked_agent_id, agent_current_pos, time_step, node, avoid_pos_list, walls):
    print(f'original solution for agent {agent_id}: {node.solution[agent_id]}')
    print(f'original solution for blocked agent {blocked_agent_id}: {node.solution[blocked_agent_id]}')
    # Get the current position of the blocked agent
    for (position, t), info in avoid_pos_list.items():
        if info['id'] == blocked_agent_id and t == time_step:
            blocked_agent_current_pos = position
    print(f'Agent {blocked_agent_id} is blocked by agent {agent_id} at time step {time_step} at position {agent_current_pos}')
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
                    print(f'---new_avoid_pos_list: {new_avoid_pos_list}---')
                    insert_blocked_agent_action = action
                    print(f'---insert_blocked_agent_action: {insert_blocked_agent_action}---')
                    # Insert the new action at the time step
                    node.solution[blocked_agent_id].insert(time_step, insert_blocked_agent_action)
                    print(f'node.solution[blocked_agent_id]: {node.solution[blocked_agent_id]}')
                    # Update the solution for the agent
                    # Firstly wait for the blocked agent to move
                    # Insert NoOp for the agent if the agent's solution is shorter than the current time step
                    while len(node.solution[agent_id]) < time_step:
                        node.solution[agent_id].append(Action.NoOp)
                    node.solution[agent_id].insert(time_step, Action.NoOp)
                    print(f'node.solution[agent_id]: {node.solution[agent_id]}')
                    while count < 3:
                        # Then find the next 2 move for the agent
                        for action_agent in Action:
                            if action_agent.type == ActionType.Move:
                                agent_destination = agent_current_pos + action_agent.agent_rel_pos
                                # Check if agent_destination is within the bounds of the walls matrix
                                if 0 <= agent_destination.x < len(walls[0]) and 0 <= agent_destination.y < len(walls):
                                    # Check if agent_destination is a wall or in avoid_positions
                                    is_destination_occupied = walls[agent_destination.y][agent_destination.x] or \
                                                                any(pos == agent_destination for pos, _ in new_avoid_pos_list.keys())
                                    print(f'---is_destination_occupied: {is_destination_occupied}---')
                                    if not is_destination_occupied:
                                        # add the position and time step to the new_avoid_pos_list
                                        new_avoid_pos_list[agent_destination, time_step+count+2] = {'id': agent_id}
                                        insert_agent_action = action_agent
                                        print(f'---insert_agent_action: {insert_agent_action}---')
                                        print(f'---count: {count}---')
                                        # Insert the new action at the time step
                                        node.solution[agent_id].insert(time_step+count+1, insert_agent_action)
                                        # The blocked agent id needs to wait for the agent to move
                                        node.solution[blocked_agent_id].insert(time_step+count+1, Action.NoOp)
                                        print(f'node.solution[agent_id]: {node.solution[agent_id]}')
                                        print(f'node.solution[blocked_agent_id]: {node.solution[blocked_agent_id]}')
                                        # Update the current position of the agent
                                        agent_current_pos = agent_destination
                        count += 1
                    # Insert the opposite action to the blocked agent so that it can return
                    node.solution[blocked_agent_id].insert(time_step+count+1, opposite_action(insert_blocked_agent_action))
                    print(f'final node.solution[block ed_agent_id]: {node.solution[blocked_agent_id]}')
                    # break the loop as long as 1 available action is found
                    break

    return node


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
