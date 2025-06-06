from domain.task import Task
from heuristic import AStarFrontier, HeuristicSpaceTimeAStar
from state import SpaceTimeState, State
import sys


def space_time_a_star(inital_state, constraints, task: Task):
    agents = inital_state.agents
    box = inital_state.boxes
    wall = inital_state.walls

    initial_time = 0
    initial_state = SpaceTimeState(agents, box, wall, initial_time, constraints, 0)
    frontier = AStarFrontier(HeuristicSpaceTimeAStar(initial_state))
    frontier.add(initial_state, task)

    explored = set()

    while not frontier.is_empty():
        current_state = frontier.pop()
        
        if current_state.is_goal_state_for_subgoal(task, current_state.agents[0]):
            plan = current_state.extract_plan()
            return plan  # Return the plan to reach the goal state

        explored.add(current_state)

        for state in current_state.get_expanded_states(task):
            if state not in explored and not frontier.contains(state): 
                frontier.add(state, task)

    # print("No solution found", file=sys.stderr)
    # print(f"\n\n=============SPACE_TIME_ASTAR - v{agents[0].value} - end===============", file=sys.stderr)
    raise RuntimeError("Space Time A* cannot find solution for this problem, check HTN")  # No solution found