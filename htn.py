# Use 2 aganet 2 boxes firstly
from state import State


def htn(initial_state):
    # Assuming initial_state is an instance of the State class

    # Create individual problems for each agent
    problems_list = []
    for agent in initial_state.agents:
        print(f"---agent--{agent}")
        # Find the box that matches the agent's color
        agent_box = next((box for box in initial_state.boxes if box.color == agent.color), None)
        print(f"---agent_box--{agent_box}")
        # Find the goal that matches the box's ID
        agent_goal = next((goal for goal in initial_state.goals if goal.id == agent_box.id), None)
        print(f"---agent_goal--{agent_goal}")
        # Create a new state with only the current agent, the matching box, and the matching goal
        individual_state = State([agent], [agent_box], [agent_goal])
        print(f"---individual_state--{individual_state}")
        problems_list.append(individual_state)

    return problems_list


