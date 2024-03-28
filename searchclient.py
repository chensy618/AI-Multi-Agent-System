import io
import argparse
import sys
import memory
from collections import namedtuple

from domain.position import Position
from domain.color import Color
from state import State
from domain.agent import Agent
from domain.box import Box
from domain.goal import Goal

# data structure for agent, box, goal
AgentConfig = namedtuple('AgentConfig', ['position', 'id', 'color'])
BoxConfig = namedtuple('BoxConfig', ['position', 'letter', 'color'])
GoalConfig = namedtuple('GoalConfig', ['position', 'letter'])

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
        # print(f"parse_layout-marker-{layout,line,marker}")
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
        print(f"---agent_colors, box_colors--{agent_colors, box_colors}")
        initial_layout, goal_layout = LevelParser.parse_initial_and_goal_states(server_messages)
        print(f"---initial_layout, goal_layout--{initial_layout, goal_layout}")

        # agents, boxes, goals initialization : empty lists
        # iterate through the initial_layout and goal_layout
        agents, boxes, goals = [], [], []
        for row_idx, row in enumerate(initial_layout):
            for col_idx, char in enumerate(row):
                position = Position(row_idx, col_idx)
                if char.isdigit():
                    agents.append(AgentConfig(position, int(char), agent_colors.get(int(char))))
                elif char.isalpha() and char.isupper():
                    boxes.append(BoxConfig(position, char, box_colors.get(char)))

        for row_idx, row in enumerate(goal_layout):
            for col_idx, char in enumerate(row):
                if char in box_colors:  # Assuming goals are only for boxes
                    goals.append(GoalConfig(Position(row_idx, col_idx), char))

        # debug print
        # print(agents, boxes, goals)
                    
        # Convert configs to actual objects
        agent_objs = [Agent(position, id_, color) for position, id_, color in agents]
        box_objs = [Box(position, letter, color) for position, letter, color in boxes]
        goal_objs = [Goal(position, letter) for position, letter in goals]

        # after implementing goal, uncomment the line below
        # return State(agent_objs, box_objs, goal_objs)
        return State(agent_objs, box_objs)

    @staticmethod
    def main(args) -> None:
        print('SearchClient initializing. I am sending this using the error output stream.', file=sys.stderr)

        print('SearchClient', flush=True)
        print('#This is a comment.', flush=True)

        server_messages = io.TextIOWrapper(sys.stdin.buffer, encoding='ASCII')
        initial_state = SearchClient.parse_level(server_messages)
        
        # TODO: Search for a plan., we need to initiate our plan to start here
        # Example plan - replace with actual search logic
        print('Starting.', file=sys.stderr, flush=True)
        plan = ['NoOp']
        if plan is None:
            print('Unable to solve level.', file=sys.stderr, flush=True)
            sys.exit(0)
        else:
            print('Found solution of length {}.'.format(len(plan)), file=sys.stderr, flush=True)  
        for joint_action in plan:
            print(joint_action, flush=True)
            #print("|".join(a.name_ + "@" + a.name_ for a in joint_action), flush=True)
            #We must read the server's response to not fill up the stdin buffer and block the server.
            response = server_messages.readline()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simple client based on state-space graph search.')
    parser.add_argument('--max-memory', metavar='<MB>', type=float, default=2048.0, help='The maximum memory usage allowed in MB (soft limit, default 2048).')
    args = parser.parse_args()

    # set the maximum memory usage allowed in MB (soft limit, default 2048)
    memory.max_usage = args.max_memory

    SearchClient.main(args)