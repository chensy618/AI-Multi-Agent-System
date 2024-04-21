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
    initial_state = SpaceTimeState(agent, box, goal, initial_time)
    print(f"---initial_state--{initial_state}")
    initial_state.constraints = constraints
    print(f"---initial_state.constraints--{initial_state.constraints}")

