from domain.agent import Agent
from domain.box import Box
from domain.color import Color
from domain.goal import Goal
from domain.position import Position
from domain.wall import Wall

class StateTranslator:
    def __init__(self, agents, boxes, goals, walls, width, height):
        self.agents = agents
        self.boxes = boxes
        self.goals = goals
        self.walls = walls
        self.width = width
        self.height = height
        self.all_locations = {f"L{x}_{y}" for x in range(1, width + 1) for y in range(1, height + 1)}
        
    def translate_position(self, position):
        return f"L{position.x}_{position.y}"    
    
    def construct_init_statements(self):
        copy_agents = [Agent(agent.pos, agent.id, agent.color) for agent in self.agents]
        copy_boxes = [Box(box.pos, box.id, box.color) for box in self.boxes]
        copy_walls = [Wall(wall.pos) for wall in self.walls]
        agent_statements = [f"AgentAt({agent.id},{self.translate_position(agent.pos)})" for agent in copy_agents]
        box_statements = [f"BoxAt({box.id},{self.translate_position(box.pos)})" for box in copy_boxes]
       
        occupied_locations = {self.translate_position(occupy.pos) for occupy in copy_agents} | {self.translate_position(occupy.pos) for occupy in copy_boxes} | {self.translate_position(occupy.pos.position) for occupy in copy_walls}
        free_locations = self.all_locations - occupied_locations
        free_statements = [f"Free({location})" for location in free_locations]
        
        # construct neighbor statements from the free locations
        neighbor_statements = []
        for location in free_locations:
            x, y = location[1:].split("_")
            x, y = int(x), int(y)
            neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
            for neighbor in neighbors:
                if 1 <= neighbor[0] <= self.width and 1 <= neighbor[1] <= self.height:
                    neighbor_statements.append(f"Neighbor({location},{self.translate_position(Position(neighbor[0], neighbor[1]))})")
                    #print(f"---neighbor_statements--{neighbor_statements}")
        statements = agent_statements + box_statements + free_statements + neighbor_statements
        return " & ".join(statements)
    
    def construct_goal_statements(self):
        copy_goals = [Goal(goal.pos, goal.id) for goal in self.goals]
        goal_statements = [f"GoalAt({goal.id},{self.translate_position(goal.pos)})" for goal in copy_goals]
        return " & ".join(goal_statements)
    
    def construct_wall_statements(self):
        copy_walls = [Wall(wall.pos) for wall in self.walls]
        wall_statements = [f"WallAt({self.translate_position(wall.pos.position)})" for wall in copy_walls]
        return " & ".join(wall_statements)
    