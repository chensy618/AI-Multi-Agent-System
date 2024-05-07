from domain.task import Task
from heuristic import AStarFrontier, HeuristicAStar
from state import SpaceTimeState, State
import sys


def space_time_a_star(inital_state, constraints, task: Task):
    agents = inital_state.agents
    print(f"\n\n=============SPACE_TIME_ASTAR - {agents[0].uid}===============", file=sys.stderr)
    box = inital_state.boxes
    goal = State.goals
    wall = inital_state.walls

    print(f"---constraints--- {constraints}", file=sys.stderr)
    initial_time = 0
    initial_state = SpaceTimeState(agents, box, wall, goal, initial_time, constraints, 0)

    frontier = AStarFrontier(HeuristicAStar(initial_state))
    frontier.add(initial_state, task)

    explored = set()

    while not frontier.is_empty():
        current_state = frontier.pop()
        if current_state.is_goal_state_for_subgoal(task, current_state.agents[0]):
            # print(f"---current_state.is_goal_state_for_subgoal()--- {current_state.is_goal_state_for_subgoal(task, current_state.agents[0])}",file=sys.stderr)
            # print(f"---current_state.joint_action()--- {current_state.joint_action}",file=sys.stderr)
            return current_state.extract_plan()  # Return the plan to reach the goal state

        explored.add(current_state)

        for state in current_state.get_expanded_states():
            if state not in explored and not frontier.contains(state):
                frontier.add(state, task)

    print("No solution found", file=sys.stderr)
    print(f"\n\n=============SPACE_TIME_ASTAR - {agents[0].uid}===============", file=sys.stderr)
    return None  # No solution found