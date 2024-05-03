from heuristic import AStarFrontier, HeuristicAStar
from state import SpaceTimeState, State


def space_time_a_star(inital_state, constraints, round):
    agent = inital_state.agents
    box = inital_state.boxes
    goal = State.goals

    initial_time = 0
    initial_state = SpaceTimeState(agent, box, goal, initial_time, constraints, 0)
    print(f"---initial_state--{initial_state}")
    print(f"---initial_state.constraints--{initial_state.constraints}")

    frontier = AStarFrontier(HeuristicAStar(initial_state))
    frontier.add(initial_state, round)

    explored = set()

    while True:
        if frontier.is_empty():
            print("No solution found")
            return None  # No solution found

        current_state = frontier.pop()
        print(f"---current_state--- {current_state.agents[0].pos}")
        if current_state.is_goal_state():
            print(f"---current_state.is_goal_state()--- {current_state.is_goal_state()}")
            print(f"---current_state.joint_action()--- {current_state.joint_action}")
            return current_state.extract_plan()  # Return the plan to reach the goal state

        explored.add(current_state)

        for state in current_state.get_expanded_states():
            if state not in explored and not frontier.contains(state):
                frontier.add(state, round)

