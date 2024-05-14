from collections import deque
import io
import argparse
import pprint
import sys
import time
from domain.action import Action
from domain.task import Task
from htn.htn_resolver import HTNResolver
import memory
from itertools import groupby

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

def goal_state_analysis(layout, r, c):
        x1 = x2 = y1 = y2 = 0
        # direct neighbour positions
        direct_neighbours = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
        # diagonal neighbour positions
        diagonal_neighbours = [(r-1, c-1), (r-1, c+1), (r+1, c-1), (r+1, c+1)]
    
        for pos in direct_neighbours:
         # Check if the direct neighbour position is within the layout boundaries
            #if 0 <= pos[0] < len(layout[0]) and 0 <= pos[1] < len(layout):
                if layout[pos[0]][pos[1]] == '+':
                    x2 = x2+1
                if layout[pos[0]][pos[1]].isdigit() or layout[pos[0]][pos[1]].isupper():
                    y2 = y2+1

        for pos in diagonal_neighbours:
            # Check if the diagonal neighbour position is within the layout boundaries
            if 0 <= pos[0] < len(layout[0]) and 0 <= pos[1] < len(layout[pos[0]]):
            #if 0 <= pos[0] < len(layout[pos[1]]) and 0 <= pos[1] < len(layout[pos[0]]):
                if layout[pos[0]][pos[1]] == '+':
                    x1 = x1+1
                if layout[pos[0]][pos[1]].isdigit() or layout[pos[0]][pos[1]].isupper():
                    y1 = y1+1
        
        x1 = x1 + x2
        y1 = y1 + y2
            
        return x1, y1, x2, y2

def neighbour_goal_analysis(goals, goal, group_number, base_score):
    x, y = goal.pos.x, goal.pos.y
    # direct neighbour positions
    direct_neighbours = [(y-1, x), (y+1, x), (y, x-1), (y, x+1)]
    # if goal.z == 0:
    #     base_score = 50
    #     group_number += 1
    #     goal.group = group_number
    neighbour_goals = []
    original_goals = []
    for g in goals:
        found = False
        for pos in direct_neighbours:
            #print(f"---pos--{pos[0], pos[1]}")
            #print(f"---g.pos--{g.pos.y, g.pos.x}")
            if pos[0] == g.pos.y and pos[1] == g.pos.x:
                #print("--wwwwwwwwwwwwwwwwwwwwwwwwwwwwtttttttttttttttttttttttttt", file=sys.stderr)
                neighbour_goals.append(g)
                #goals.remove(g)
                found = True
        if found == False:    
            original_goals.append(g)
        
    if neighbour_goals==[]:
        #print("--ytttttttttttttttttttttttttttttttttttttttttttttttttttttt", file=sys.stderr)
        return original_goals, neighbour_goals, group_number, base_score
    else:
        #neighbour_goals = sorted(neighbour_goals , key=lambda neighbour_goals: (-neighbour_goals.x2, -neighbour_goals.y2, -neighbour_goals.x1, -neighbour_goals.y1))
        for g in neighbour_goals:
            g.group = goal.group
            g.z = base_score-1
            base_score -= 1
        return original_goals, neighbour_goals, group_number, base_score

def post_goal_state_analysis(goals):
    sorted_goals = sorted(goals, key=lambda available_goals: (-available_goals.x2, -available_goals.y2, -available_goals.x1, -available_goals.y1))
    frontier_neighbour_goals = []
    group_number = 0
    base_score = 0
    while sorted_goals:
        goal = sorted_goals[0]
        sorted_goals.pop(0)
        if goal.z == 0:
            base_score = 50
            group_number += 1
            goal.group = group_number
        sorted_goals, neighbour_goals, group_number, base_score = neighbour_goal_analysis(sorted_goals, goal, group_number, base_score)
        if neighbour_goals: frontier_neighbour_goals.extend(neighbour_goals)
        while frontier_neighbour_goals:
            for current_goal in frontier_neighbour_goals:
                sorted_goals, goal, group_number, base_score = neighbour_goal_analysis(sorted_goals, current_goal, group_number, base_score)
                frontier_neighbour_goals.remove(current_goal)
                #base_score -= 1
                if goal:    
                    frontier_neighbour_goals.extend(goal)

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

        box_uid = 0
        for row_idx, row in enumerate(initial_layout):
            for col_idx, char in enumerate(row):
                position = Position(col_idx, row_idx)
                if char.isdigit():
                    agents.append(Agent(pos=position, value=int(char), color=agent_colors.get(int(char))))
                elif char.isupper():
                    boxes.append(Box(pos=position, value=char, uid=box_uid, color=box_colors.get(char)))
                    box_uid += 1
                else:
                    if char == '+':
                        walls[row_idx][col_idx] = True

        # read position of goals
        goal_uid = 0

        for row_idx, row in enumerate(goal_layout):
            for col_idx, char in enumerate(row):
                if char.isdigit() or char.isupper():
                    x1, y1, x2, y2 = goal_state_analysis(goal_layout, row_idx, col_idx)
                    goals.append(Goal(pos=Position(col_idx, row_idx), value=char, uid=goal_uid, x1=x1, y1=y1, x2=x2, y2=y2))
                    goal_uid += 1
        post_goal_state_analysis(goals)


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

        print("\n========INITIAL STATE========\n", file=sys.stderr)
        for goal in State.goals:
            State.goal_map[goal.uid] = State.initialize_goal_map(initial_state.walls, goal.pos)
            print(f"Goal - v{goal.value} ---> ", goal, file=sys.stderr)
        for agent in initial_state.agents:
            print(f"Agent - v{agent.value} ---> ", agent, file=sys.stderr)
        for box in initial_state.boxes.values():
            State.box_goal_map[box.uid] = State.initialize_goal_map(initial_state.walls, box.pos)
            print(f"Box - v{box.value} ---> ", box, file=sys.stderr)

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

        print("\n========INITIAL STATE========\n", file=sys.stderr)

        resolver = HTNResolver()
        resolver.initialize_problems(initial_state)

        final_plan = []
        current_state = initial_state
        while(resolver.has_any_task_left(current_state)):
            print("Round -> ", resolver.round_counter, file=sys.stderr)
            resolver.create_round(current_state)

            print(resolver.round, file=sys.stderr)

            plan = conflict_based_search(current_state, resolver.round)
            if plan is None:
                continue
            for time_step in plan:
                print("Time step -> ", time_step, file=sys.stderr)
                print("Plan -> ", plan, file=sys.stderr)
                final_plan.append(time_step)
                current_state = current_state.result(time_step)


        print("========PROBLEM========\n", file=sys.stderr)

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
