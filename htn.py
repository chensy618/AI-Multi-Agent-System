# Use 2 aganet 2 boxes firstly
# Doesn't work for agents only yet

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
        if agent_box:
            agent_goal = next((goal for goal in initial_state.goals if goal.id == agent_box.id), None)
            print(f"---agent_goal--{agent_goal}")
            individual_state = State([agent], [agent_box], [agent_goal])
        else:
            print(f"---initial_state.goals--{initial_state.goals}")
            agent_goal = next((goal for goal in initial_state.goals if int(goal.id) == int(agent.id)), None)
            print(f"---agent_goal--{agent_goal}")
            individual_state = State([agent], [], [agent_goal])
        problems_list.append(individual_state)
    return problems_list


