import argparse
import io
import sys
import time

import cProfile

import memory
from domain.color import Color
from state import State

class SearchClient:

    # TODO: parse_level with out objects defined in domain... (Agent, Box, Goal)
    @staticmethod
    def parse_level(server_messages) -> 'State':
        # We can assume that the level file is conforming to specification, since the server verifies this.
        # Read domain.
        server_messages.readline() # #domain
        server_messages.readline() # hospital
        
        # Read Level name.
        server_messages.readline() # #levelname
        server_messages.readline() # <name>
        
        # Read colors.
        server_messages.readline() # #colors
        agent_colors = [None for _ in range(10)]
        box_colors = [None for _ in range(26)]
        line = server_messages.readline()
        while not line.startswith('#'):
            split = line.split(':')
            color = Color.from_string(split[0].strip())
            entities = [e.strip() for e in split[1].split(',')]
            for e in entities:
                if '0' <= e <= '9':
                    agent_colors[ord(e) - ord('0')] = color
                elif 'A' <= e <= 'Z':
                    box_colors[ord(e) - ord('A')] = color
            line = server_messages.readline()
        
        # Read initial state.
        # line is currently "#initial".
        num_rows = 0
        num_cols = 0
        level_lines = []
        line = server_messages.readline()
        while not line.startswith('#'):
            level_lines.append(line)
            num_cols = max(num_cols, len(line))
            num_rows += 1
            line = server_messages.readline()

        num_agents = 0
        agent_rows = [None for _ in range(10)]
        agent_cols = [None for _ in range(10)]
        walls = [[False for _ in range(num_cols)] for _ in range(num_rows)]
        boxes = [['' for _ in range(num_cols)] for _ in range(num_rows)]
        row = 0
        for line in level_lines:
            for col, c in enumerate(line):
                if '0' <= c <= '9':
                    agent_rows[ord(c) - ord('0')] = row
                    agent_cols[ord(c) - ord('0')] = col
                    num_agents += 1
                elif 'A' <= c <= 'Z':
                    boxes[row][col] = c
                elif c == '+':
                    walls[row][col] = True
            
            row += 1
        del agent_rows[num_agents:]
        del agent_rows[num_agents:]
        
        # Read goal state.
        # line is currently "#goal".
        goals = [['' for _ in range(num_cols)] for _ in range(num_rows)]
        line = server_messages.readline()
        row = 0
        while not line.startswith('#'):
            for col, c in enumerate(line):
                if '0' <= c <= '9' or 'A' <= c <= 'Z':
                    goals[row][col] = c
            
            row += 1
            line = server_messages.readline()
        
        # End.
        # line is currently "#end".
        
        # TODO: This should be moved to State class, and implemented as static variables not outside here
        State.walls = walls
        State.goals = goals

        # TODO: agents should be list of Agent, boxes should be list of Boxes
        return State(agents, boxes)

    @staticmethod
    def main(args) -> None:
        # Use stderr to print to the console.
        print('SearchClient initializing. I am sending this using the error output stream.', file=sys.stderr, flush=True)
        
        # Send client name to server.
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding='ASCII')
        print('SearchClient', flush=True)
        
        # We can also print comments to stdout by prefixing with a #.
        print('#This is a comment.', flush=True)
        
        # Parse the level.
        server_messages = sys.stdin
        if hasattr(server_messages, "reconfigure"):
            server_messages.reconfigure(encoding='ASCII')
        initial_state = SearchClient.parse_level(server_messages)
        
        # TODO: Search for a plan., we need to initiate our plan to start here
        print('Starting.', file=sys.stderr, flush=True)
        plan = 'To be continued...'
        
        # Print plan to server.
        if plan is None:
            print('Unable to solve level.', file=sys.stderr, flush=True)
            sys.exit(0)
        else:
            print('Found solution of length {}.'.format(len(plan)), file=sys.stderr, flush=True)
            
            for joint_action in plan:
                print("|".join(a.name_ + "@" + a.name_ for a in joint_action), flush=True)
                # We must read the server's response to not fill up the stdin buffer and block the server.
                response = server_messages.readline()

if __name__ == '__main__':
    # Program arguments.
    parser = argparse.ArgumentParser(description='Simple client based on state-space graph search.')
    parser.add_argument('--max-memory', metavar='<MB>', type=float, default=2048.0, help='The maximum memory usage allowed in MB (soft limit, default 2048).')
    
    args = parser.parse_args()
    
    # Set max memory usage allowed (soft limit).
    memory.max_usage = args.max_memory
    
    # Run client.
    SearchClient.main(args)
