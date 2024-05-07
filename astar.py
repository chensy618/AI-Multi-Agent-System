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
    print("\n=============ASTAR===============", file=sys.stderr)
    print(f"---initial_state---{initial_state.boxes} {task}", file=sys.stderr)
    frontier = AStarFrontier(HeuristicAStar(initial_state))
    frontier.add(initial_state, task)
    
    explored = set()

    while not frontier.is_empty():
        current_state = frontier.pop()
        # print(f"---current_state--- {[agent.pos for agent in current_state.agents]}", file=sys.stderr)

        # print("current_state.agents: " + current_state.agents[0].uid, file=sys.stderr)

        if current_state.is_goal_state_for_subgoal(task, current_state.agents[0]):
            plan = current_state.extract_plan()
            print(f"Extracted plan: {plan}", file=sys.stderr)
            return plan  # Return the plan to reach the goal state

        explored.add(current_state)

        for state in current_state.get_expanded_states():
            if state not in explored and not frontier.contains(state):
                frontier.add(state, task)

        if len(explored) % 1000 == 0:
            print_search_status(explored, frontier)

        if memory.get_usage() > memory.max_usage:
            print_search_status(explored, frontier)
            print('Maximum memory usage exceeded.', file=sys.stderr)
            return None
        
    print("No solution found", file=sys.stderr)
    print("=============ASTAR===============\n", file=sys.stderr)
    return None  # No solution found
    
def print_search_status(explored, frontier):
    status_template = '#Expanded: {:8,}, #Frontier: {:8,}, #Generated: {:8,}, Time: {:3.3f} s\n[Alloc: {:4.2f} MB, MaxAlloc: {:4.2f} MB]'
    elapsed_time = time.perf_counter() - start_time
    print(status_template.format(len(explored), frontier.size(), len(explored) + frontier.size(), elapsed_time, memory.get_usage(), memory.max_usage), file=sys.stderr)