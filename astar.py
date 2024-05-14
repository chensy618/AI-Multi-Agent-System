import time
import sys
from heuristic import AStarFrontier, HeuristicAStar
import memory
from domain.action import Action
import time
import sys
import memory

globals().update(Action.__members__)
start_time = time.perf_counter()

def astar(initial_state, task):
    print(f"\n=============ASTAR - v{initial_state.agents[0].value}===============", file=sys.stderr)
    # print(f'------------------------initial agent state: {initial_state.agents}------------------------', file=sys.stderr)
    # print(f'------------------------initial box state: {initial_state.boxes}------------------------', file=sys.stderr)
    # print(f'------------------------initial goal state: {initial_state.goals}------------------------', file=sys.stderr)
    frontier = AStarFrontier(HeuristicAStar(initial_state))
    frontier.add(initial_state, task)
    explored = set()
    while not frontier.is_empty():
        current_state = frontier.pop()

        if current_state.is_goal_state_for_subgoal(task, current_state.agents[0]):
            plan = current_state.extract_plan()
            print(f"Extracted plan: {plan}", file=sys.stderr)
            print(f"=============ASTAR - v{initial_state.agents[0].value}===============\n", file=sys.stderr)
            return plan  # Return the plan to reach the goal state

        explored.add(current_state)

        for state in current_state.get_expanded_states(task):
            if state not in explored and not frontier.contains(state):
                frontier.add(state, task)

        if len(explored) % 1000 == 0:
            print_search_status(explored, frontier)

        if memory.get_usage() > memory.max_usage:
            print_search_status(explored, frontier)
            print('Maximum memory usage exceeded.', file=sys.stderr)
            return None

    print("No solution found", file=sys.stderr)
    print(f"\n=============ASTAR - v{initial_state.agents[0].value}===============", file=sys.stderr)
    #raise RuntimeError("A* cannot find solution for this problem, check HTN")  # No solution found
    return None

def print_search_status(explored, frontier):
    status_template = '#Expanded: {:8,}, #Frontier: {:8,}, #Generated: {:8,}, Time: {:3.3f} s\n[Alloc: {:4.2f} MB, MaxAlloc: {:4.2f} MB]'
    elapsed_time = time.perf_counter() - start_time
    print(status_template.format(len(explored), frontier.size(), len(explored) + frontier.size(), elapsed_time, memory.get_usage(), memory.max_usage), file=sys.stderr)
