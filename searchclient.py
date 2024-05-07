import io
import argparse
import pprint
import sys
from domain.action import Action
from htn.htn_resolver import HTNResolver
import memory

from cbs.cbs import conflict_based_search
from domain.position import Position
from domain.color import Color
from state import State
from domain.agent import Agent
from domain.box import Box
from domain.goal import Goal

# import debugpy
# debugpy.listen(("localhost", 12345)) # Open a debugging server at localhost:1234
# debugpy.wait_for_client() # Wait for the debugger to connect
# debugpy.breakpoint() # Ensure the program starts paused
# import debugpy
# debugpy.listen(("localhost", 1234)) # Open a debugging server at localhost:1234
# debugpy.wait_for_client() # Wait for the debugger to connect
# debugpy.breakpoint() # Ensure the program starts paused

# data structure for agent, box, goal
layout_rows = 0
layout_cols = 0
class LevelParser:
    @staticmethod
    def parse_colors(server_messages):
        agent_colors, box_colors = {}, {}
        # read "#colors" line
        line = server_messages.readline()
        # print(f"---parse_colors--{line}")

        # read the color names and entities
        line = server_messages.readline()
        while not line.startswith('#'):
            color_name, entities = map(str.strip, line.split(':'))
            # print(f"--get-Color-name--: {color_name}")
            color = Color.from_string(color_name)
            # print(f"----Color---: {color,color_name}, entities: {entities}")
            for entity in entities.split(','):
                entity = entity.strip()  # Strip spaces from the entity
                if entity.isdigit():
                    agent_colors[int(entity)] = color
                else:
                    box_colors[entity] = color
            # read next line, or line is at "initial"
            line = server_messages.readline()
        return agent_colors, box_colors

    @staticmethod
    # read the layout of the level
    # parametres: server_messages, marker
    # marker: #goal or #end, read until the marker
    # return: layout, line
    def parse_layout(server_messages, marker):
        layout = []
        line = server_messages.readline()
        while not line.startswith(marker):
            layout.append(line.rstrip('\n'))
            line = server_messages.readline()
            #print(f"parse_layout-marker-{layout,line,marker}")
        return layout, line

    @staticmethod
    # get the initial and goal states
    def parse_initial_and_goal_states(server_messages):
        initial_layout, line = LevelParser.parse_layout(server_messages, '#goal')
        goal_layout, _ = LevelParser.parse_layout(server_messages, '#end')
        # print(f"parse_initial and goal--{initial_layout,goal_layout}")
        return initial_layout, goal_layout

    @staticmethod
    # get the initial state
    def parse_initial(server_messages):
        initial_layout, line = LevelParser.parse_layout(server_messages, '#goal')
        return initial_layout
    @staticmethod
    # get the goal state
    def parse_goal(server_messages):
        goal_layout, line = LevelParser.parse_layout(server_messages, '#end')
        return goal_layout
    
class SearchClient:
    @staticmethod
    def parse_level(server_messages) -> 'State':
        for _ in range(4):  # Skip domain and level name headers
            server_messages.readline()

        agent_colors, box_colors = LevelParser.parse_colors(server_messages)
        # print(f"---agent_colors, box_colors--{agent_colors, box_colors}")
        initial_layout, goal_layout = LevelParser.parse_initial_and_goal_states(server_messages)
        # print(f"---initial_layout, goal_layout--{initial_layout, goal_layout}", file=sys.stderr)
        #print(f"---initial_layout, goal_layout--{initial_layout, goal_layout}")
        # agents, boxes, goals, walls initialization : empty lists
        # iterate through the initial_layout and goal_layout
        agents, boxes, goals = [], [], []

        nrows = len(initial_layout)
        # ncols = len(initial_layout[0]) if nrows > 0 else 0
        ncols = max(len(row) for row in initial_layout)
        # print(f"---nrows, ncols--{nrows, ncols}")
        walls = [[False] * ncols for _ in range(nrows)]
        # print(f"---walls--{walls[0]}")

        agent_uid = 0
        box_uid = 0
        for row_idx, row in enumerate(initial_layout):
            for col_idx, char in enumerate(row):
                position = Position(col_idx, row_idx)
                if char.isdigit():
                    agents.append(Agent(pos=position, value=int(char), uid=agent_uid, color=agent_colors.get(int(char))))
                    agent_uid += 1
                elif char.isupper():
                    boxes.append(Box(pos=position, value=char, uid=box_uid, color=box_colors.get(char)))
                    box_uid += 1
                else:
                    if char == '+':
                        walls[row_idx][col_idx] = True
                        # print(f"---row, col--{row_idx, col_idx}")

        # read position of goals
        goal_uid = 0

        for row_idx, row in enumerate(goal_layout):
            for col_idx, char in enumerate(row):
                if char.isdigit() or char.isupper():
                    goals.append(Goal(pos=Position(col_idx, row_idx), value=char, uid=goal_uid))
                    goal_uid += 1
                    

        # Calculate the dimensions of the level\
        global layout_rows, layout_cols
        layout_rows = len(initial_layout)
        layout_cols = max(len(row) for row in initial_layout)

        State.goals = goals

        box_map = {box.uid: box for box in boxes}
        return State(agents, box_map, walls)

    @staticmethod
    def main(args) -> None:
        print('SearchClient', flush=True)
        # print('#This is a comment.', flush=True)

        server_messages = io.TextIOWrapper(sys.stdin.buffer, encoding='ASCII')
        initial_state = SearchClient.parse_level(server_messages)

        print("\nINITIAL STATE", file=sys.stderr)
        for agent in initial_state.agents:
            print(f"Agent - {agent.value} ---> ", agent, file=sys.stderr)
        for box in initial_state.boxes.values():
            print(f"Box - {box.value} ---> ", box, file=sys.stderr)
        for goal in State.goals:
            print(f"Goal - {goal.value} ---> ", goal, file=sys.stderr)
        print("INITIAL STATE\n", file=sys.stderr)

        for goal in State.goals:
            print(f"Goal - {goal.value} ---> ", goal, file=sys.stderr)
            State.goal_map[goal.uid] = State.initialize_goal_map(initial_state.walls, goal.pos)
        for box in initial_state.boxes.values():
            State.box_goal_map[box.uid] = State.initialize_goal_map(initial_state.walls, box.pos)
        
        for goal_id in State.goal_map.keys():
            print(f"\n----------Distance map for Goal - {goal_id}-------------", file=sys.stderr)
            goal_grid = State.goal_map[goal_id]
            for row in goal_grid:
                print(' '.join(f"{cell if cell is not None else 'None':4}" for cell in row), file=sys.stderr)

        for box_id in State.box_goal_map.keys():
            print(f"\n----------Distance map for Box - {box_id}-------------", file=sys.stderr)
            goal_grid = State.box_goal_map[box_id]
            for row in goal_grid:
                print(' '.join(f"{cell if cell is not None else 'None':4}" for cell in row), file=sys.stderr)

        ### PSEUDO CODE for integrating CBS and HTN

        # Initialize problems for each agent
        # - Get same colored agents and boxes
        # - Distribute the boxes between those agents
        # - Store all tasks in a Map<agentId -> deque<Task>>
        # - calculate BFS_from_goal / BFS_from_box for each goal / box: goal_map and box_map Map<goalId -> [][]>, Map<boxId -> [][]>
        #   -> it has to be run in a state with only walls and free cells
        
        print("-----------Problem-------------\n", file=sys.stderr)
        resolver = HTNResolver()
        resolver.initialize_problems(initial_state)

        print(f"---agent_tasks---{resolver.agent_tasks}", file=sys.stderr)

        final_plan = None
        current_state = initial_state
        while(resolver.has_any_task_left()):
            print("Round -> ", resolver.round_counter, file=sys.stderr)
            resolver.create_round()
            
            plan = conflict_based_search(current_state, resolver.round)
            final_plan = plan
            # - final_plan.append(plan)
            # set current_state to the state after executing the plan.

        print("-----------Problem-------------\n", file=sys.stderr)


        # for time_step in final_plan:
        #     time_step.append(Action.NoOp)

        if final_plan is None:
            print('Unable to solve level.', file=sys.stderr, flush=True)
            sys.exit(0)
        else:
            print('Found solution of length {}.'.format(len(plan)), file=sys.stderr, flush=True)
            for joint_action in final_plan:
                print("|".join(a.name_ + "@" + a.name_ for a in joint_action), flush=True)
                #We must read the server's response to not fill up the stdin buffer and block the server.
                response = server_messages.readline()
                # print(f"---response--{response}")


        # Prioritize tasks based on plausability and factor of blocking
        # - the tasks whose goal doesn't block any goal should be given more priority (priority + 100)
        # - If A* can solve a task it should be prioritized over the tasks that can't be solved (priority + 10)
        # - the closest box to the agent should be given priority, if we can't choose between multiple boxes (priority + x, where x is based on the number of tasks with the same priority) 

        # Heuristic function should change according to task priority
        # - h(n) = w1 * (distance_of_agent_to_box + distance_of_box_to_goal), (this is for the box, which is on priority)
        #   -> w1 should be set based on priority (e.g.: if an agent is blocking other agents it should be given high priority w1 = 100, w2 = 10, w3 = 1...)
        #   distance_of_agent_to_box is based on whether box has moved away, if it has moved we return manhattan_distance otherwise perfect heuristic from box_map
        
        # Integrate CBS with HTN
        # - initialize agent_current_tasks
        # - current_state = initial_state
        # - While loop: (current_tasks is empty)
            # - If an agent has a task left, assign priority task to agent
            # - Create the plans for each agent
            # - solution = run_CBS(current_state, agents),
            # - joint_actions.append(solution)
            # - current_state = state_after_executing_the_solution

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simple client based on state-space graph search.')
    parser.add_argument('--max-memory', metavar='<MB>', type=float, default=2048.0, help='The maximum memory usage allowed in MB (soft limit, default 2048).')
    args = parser.parse_args()

    # set the maximum memory usage allowed in MB (soft limit, default 2048)
    memory.max_usage = args.max_memory

    SearchClient.main(args)
