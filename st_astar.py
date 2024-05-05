from heuristic import AStarFrontier, HeuristicAStar
from state import SpaceTimeState, State
import sys


def space_time_a_star(inital_state, constraints, round):
    agent = inital_state.agents
    box = inital_state.boxes
    goal = State.goals
    wall = inital_state.walls

    initial_time = 0
    initial_state = SpaceTimeState(agent, box, wall, goal, initial_time, constraints, 0)
    print(f"---initial_state--{initial_state}",file=sys.stderr)
    print(f"---initial_state.constraints--{initial_state.constraints}",file=sys.stderr)

    frontier = AStarFrontier(HeuristicAStar(initial_state))
    frontier.add(initial_state, round)

    explored = set()

    while True:
        if frontier.is_empty():
            print("No solution found", file=sys.stderr)
            return None  # No solution found

        current_state = frontier.pop()
        print(f"---current_state--- {current_state.agents[0].pos}",file=sys.stderr)
        if current_state.is_goal_state():
            print(f"---current_state.is_goal_state()--- {current_state.is_goal_state()}",file=sys.stderr)
            print(f"---current_state.joint_action()--- {current_state.joint_action}",file=sys.stderr)
            return current_state.extract_plan()  # Return the plan to reach the goal state

        explored.add(current_state)

        for state in current_state.get_expanded_states():
            if state not in explored and not frontier.contains(state):
                frontier.add(state, round)