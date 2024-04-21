from astar import Frontier, FrontierBestFirst, PriorityQueue, Heuristic, HeuristicAStar, HeuristicWeightedAStar
from state import SpaceTimeState


def space_time_a_star(problem, constraints):
    print(f"---problem--{problem}")
    agent = problem.agents
    box = problem.boxes
    goal = problem.goals

    # print(f"---agent--{agent}")
    # print(f"---box--{box}")
    # print(f"---goal--{goal}")
    # ---agent--[Agent(pos=Position(x=2, y=1), id=0, color=Color.Red)]
    # ---box--[Box(pos=Position(x=2, y=2), id=A, color=Color.Red)]
    # ---goal--[Goal(pos=Position(x=2, y=4), id=A)]

    initial_time = 0
    initial_state = SpaceTimeState(agent, box, goal, initial_time, constraints)
    print(f"---initial_state--{initial_state}")
    print(f"---initial_state.constraints--{initial_state.constraints}")

    frontier = FrontierBestFirst(HeuristicAStar(initial_state))
    frontier.add(initial_state)

    explored = set()

    while True:
        if frontier.is_empty():
            print("No solution found")
            return None  # No solution found

        current_state = frontier.pop()
        print(f"---current_state--- {current_state.agents[0].pos}")

        if current_state.is_goal_state():
            return current_state.extract_plan()  # Return the plan to reach the goal state

        explored.add(current_state)

        for state in current_state.get_expanded_states():
            if state not in explored and not frontier.contains(state):
                frontier.add(state)

