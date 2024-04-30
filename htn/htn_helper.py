import sys
from helper.distance_calc import DistanceCalc
from state import State


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
        for box in boxes:
            if box.color not in boxes_by_color:
                boxes_by_color[box.color] = []
            boxes_by_color[box.color].append(box)
        return boxes_by_color
    
    @staticmethod
    def get_closest_goal_uid_to_box(box):
        min_dist = float('inf')
        closest_goal = None

        box_goals = [goal for goal in State.goals if (goal.value == box.value)]
        for goal in box_goals:
            dist = State.goal_map[goal.uid][box.pos.y][box.pos.x]
            if dist < min_dist:
                min_dist = dist
                closest_goal = goal

        return closest_goal.uid
    
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