import sys
from typing import List
from domain.action import Action
from domain.box import Box
from domain.position import Position
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
            if dist is not None and dist < min_dist:
                min_dist = dist
                closest_goal_uid = goal_uid

        return closest_goal_uid
    
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
    
    @staticmethod
    def get_list_positions_from_actions(actions: list[Action], pos: Position) -> 'list[Position]':
        current_position = pos

        positions = []

        for action in actions:
            current_position = action.get_position() + current_position
            positions.append(current_position)

        return positions