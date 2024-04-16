import io
import argparse
import sys
from cbs.cbs import conflict_based_search
from htn.htn_resolver import HTNResolver
import memory
from collections import namedtuple
from domain.position import Position
from domain.color import Color
from state import State
from domain.agent import Agent
from domain.box import Box
from domain.goal import Goal
from domain.wall import Wall
from pathfinding import SpaceTimeAstar
from pop.action_schema import StateTranslator

# import debugpy
# debugpy.listen(("localhost", 12345)) # Open a debugging server at localhost:1234
# debugpy.wait_for_client() # Wait for the debugger to connect
# debugpy.breakpoint() # Ensure the program starts paused
# import debugpy
# debugpy.listen(("localhost", 1234)) # Open a debugging server at localhost:1234
# debugpy.wait_for_client() # Wait for the debugger to connect
# debugpy.breakpoint() # Ensure the program starts paused

# data structure for agent, box, goal
AgentConfig = namedtuple('AgentConfig', ['position', 'id', 'color'])
BoxConfig = namedtuple('BoxConfig', ['position', 'letter', 'color'])
GoalConfig = namedtuple('GoalConfig', ['position', 'letter'])
WallConfig = namedtuple('WallConfig', ['position'])
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
        #print(f"---initial_layout, goal_layout--{initial_layout, goal_layout}")
        # agents, boxes, goals, walls initialization : empty lists
        # iterate through the initial_layout and goal_layout
        agents, boxes, goals = [], [], []

        nrows = len(initial_layout)
        ncols = len(initial_layout[0]) if nrows > 0 else 0

        walls = [[False] * ncols for _ in range(nrows)]
        for row_idx, row in enumerate(initial_layout):
            for col_idx, char in enumerate(row):
                position = Position(col_idx, row_idx)
                if char.isdigit():
                    agents.append(AgentConfig(position, int(char), agent_colors.get(int(char))))
                elif char.isupper():
                    boxes.append(BoxConfig(position, char, box_colors.get(char)))
                else:
                    if char == '+':
                        walls[row_idx][col_idx] = True

        # read position of goals
        for row_idx, row in enumerate(goal_layout):
            for col_idx, char in enumerate(row):
                if char.isdigit() or char.isupper():
                    goals.append(GoalConfig(Position(col_idx, row_idx), char))

        # Calculate the dimensions of the level\
        global layout_rows, layout_cols
        layout_rows = len(initial_layout)
        layout_cols = max(len(row) for row in initial_layout)
        print(f"---num_rows, num_cols--{layout_rows, layout_cols}")

        # Convert configs to actual objects
        agent_objs = [Agent(position, id_, color) for position, id_, color in agents]
        box_objs = [Box(position, letter, color) for position, letter, color in boxes]
        goal_objs = [Goal(position, letter) for position, letter in goals]

        # print(f"---agent_objs is--{agent_objs}")
        # print(f"---box_objs is--{box_objs}")
        # print(f"---goal_objs is--{goal_objs}")
        # print(f"---wall_objs is--{wall_objs}")

        return State(agent_objs, box_objs, goal_objs, walls)

    @staticmethod
    def main(args) -> None:
        print('SearchClient initializing. I am sending this using the error output stream.', file=sys.stderr)

        print('SearchClient', flush=True)
        # print('#This is a comment.', flush=True)

        server_messages = io.TextIOWrapper(sys.stdin.buffer, encoding='ASCII')
        initial_state = SearchClient.parse_level(server_messages)

        print(f"---initial_state--{initial_state.agents,initial_state.boxes,initial_state.goals}")
        # Search for a plan
        conflict = None
        print('Starting.', file=sys.stderr, flush=True)
        grid = [[None for _ in range(layout_cols)] for _ in range(layout_rows)]
        st_astar = SpaceTimeAstar(grid, initial_state.agents, initial_state.boxes, initial_state.goals, initial_state.walls, layout_rows, layout_cols)
        # st_astar.initialize_grid(layout_rows, layout_cols, max_time)
        plan, time_path = st_astar.st_astar(initial_state)
        # st_astar = SpaceTimeAstar(initial_state,initial_state.goals)
        # plan = st_astar.st_astar_search()
        # reservations = st_astar.get_reservation_table()
        # print(f"---reservations--{reservations}")
        print(f"time-path{time_path}")
        print(f"Plan:{plan}")
        if plan is None:
            print('Unable to solve level.', file=sys.stderr, flush=True)
            sys.exit(0)
        else:
            print('Found solution of length {}.'.format(len(plan)), file=sys.stderr, flush=True)
            for joint_action in plan:
                print("|".join(a.name_ + "@" + a.name_ for a in joint_action), flush=True)
                #We must read the server's response to not fill up the stdin buffer and block the server.
                response = server_messages.readline()
                # print(f"---response--{response}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simple client based on state-space graph search.')
    parser.add_argument('--max-memory', metavar='<MB>', type=float, default=2048.0, help='The maximum memory usage allowed in MB (soft limit, default 2048).')
    args = parser.parse_args()

    # set the maximum memory usage allowed in MB (soft limit, default 2048)
    memory.max_usage = args.max_memory

    SearchClient.main(args)
