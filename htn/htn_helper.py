import sys
from domain.box import Box
from helper.distance_calc import DistanceCalc
from state import State

def is_movable(layout, box):
        # the possible neighbour positions
        neighbours = [(box.pos.y-1, box.pos.x), (box.pos.y+1, box.pos.x), (box.pos.y, box.pos.x-1), (box.pos.y, box.pos.x+1)]
        blocked = 0
        for ny, nx in neighbours:
            # Check if the neighbour position is within the layout boundaries
            if 0 <= nx < len(layout[0]) and 0 <= ny < len(layout):
                # Check if the neighbour cell is a wall ('+') or another box
                if layout[ny][nx] == '+' or layout[ny][nx].isupper():
                   blocked += 1
                   if blocked == 4:
                       return False 
        return True

class HTNHelper:
    @staticmethod
    def categorize_agents_by_color(agents):
        agents_by_color = {}
        for agent in agents:
            if agent.color not in agents_by_color:
                agents_by_color[agent.color] = []
            agents_by_color[agent.color].append(agent)
        return agents_by_color
    
    @staticmethod
    def categorize_boxes_by_color(boxes):
        boxes_by_color = {}
        for box in boxes.values():
            if box.color not in boxes_by_color:
                boxes_by_color[box.color] = []
            boxes_by_color[box.color].append(box)
        return boxes_by_color
    
    @staticmethod
    def get_closest_goal_uid_to_box(box: Box):
        min_dist = float('inf')
        closest_goal_uid = None

        available_box_goals_uids = [goal.uid for goal in State.goals if goal.value == box.value]

        for goal_uid in available_box_goals_uids:
            dist = State.goal_map[goal_uid][box.pos.y][box.pos.x]
            if dist < min_dist:
                min_dist = dist
                closest_goal_uid = goal_uid

        return closest_goal_uid
    
    def prioritize_goals_by_difficulty(boxes):
        available_goals = [goal for box in boxes for goal in State.goals if goal.value == box.value]
        # Sort boxes by x2 in descending order and then by y2 in descending order
        sorted_goals = sorted(available_goals, key=lambda available_goals: (-available_goals.x2, -available_goals.y2))
        first_priority_goal = sorted_goals[0]
        return first_priority_goal
    
    def prioritize_boxes_by_difficulty(boxes, goal):
        available_boxes = [box for box in boxes if goal.value == box.value]
        first_priority_box = available_boxes[0]
        return first_priority_box
    
    def get_closest_goal_uid_to_agent(agent):
        min_dist = float('inf')
        closest_goal_uid = None
        
        # Filter box_goals_uids to exclude those that are already in existing_goal_uids
        available_agent_goals_uids = [goal.uid for goal in HTNHelper.agent_goals() if int(goal.value) == agent.value]
        for goal_uid in available_agent_goals_uids:
            dist = State.goal_map[goal_uid][agent.pos.y][agent.pos.x]
            if dist < min_dist:
                min_dist = dist
                closest_goal_uid = goal_uid

        return closest_goal_uid
    
    @staticmethod
    def agent_goals():
        return [goal for goal in State.goals if goal.value.isdigit()]

    @staticmethod
    def get_closest_box_uid_to_agent(agent_boxes, agent):
        min_dist = float('inf')
        closest_box = None
        for box in agent_boxes:
            dist = DistanceCalc.pos_to_box_distance(box, agent.pos)
            if dist < min_dist:
                min_dist = dist
                closest_box = box

        return closest_box.uid