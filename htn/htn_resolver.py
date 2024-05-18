from collections import deque
import sys

from domain.task import Task
from helper.distance_calc import DistanceCalc
from htn.htn_helper import HTNHelper
from state import State


class HTNResolver:
    completed_tasks = {}

    def __init__(self, initial_state: State):
        # Stores current assigned task for an agent
        self.round = {}

        # Target for an agent
        self.target = {}

        self.round_counter = 0

        agents = initial_state.agents
        boxes = initial_state.boxes

        # Stores list of Box for each Colour
        self.agents_by_color = HTNHelper.categorize_agents_by_color(agents)

        # Stores list of Agent for each Colour
        self.boxes_by_color = HTNHelper.categorize_boxes_by_color(boxes)

        self.reachability_matrix = self.initialize_reachability(initial_state)     

    def create_round(self, current_state):
        self.round_counter += 1

        boxes = current_state.boxes

        for color, agents_by_color in self.agents_by_color.items():
            boxes = self.boxes_by_color.get(color, [])

            for agent in agents_by_color:
                # target_goal_for_agent = target[agent.value]
                target_goal_for_agent = 3
                # Check if we have box
                if boxes:
                    min_box, goal_uid = min(
                        [
                            (
                                box, 
                                HTNHelper.get_closest_goal_uid_to_box(box)
                            ) 
                            for box in boxes if HTNHelper.get_closest_goal_uid_to_box(box) is not None
                        ],
                        key=lambda item: DistanceCalc.calculate_box_task(item[0], agent, item[1])
                    )
                    self.boxes_by_color[agent.color].remove(min_box)
                    if (goal_uid is None):
                        continue
                    self.round[agent.value] = Task(min_box.uid, min_box.value, goal_uid)
                else:
                    goal_uid = HTNHelper.get_closest_goal_uid_to_agent(agent)
                    if goal_uid is None:
                        continue
                    self.round[agent.value] = Task(-1, None, goal_uid)
                    print(f"self.round[{agent.value}] = Task(-1, None, {goal_uid})")
        
    def initialize_reachability(self, initial_state: State):
        boxes = initial_state.boxes
        reachability_matrix = [[[False] * len(boxes.values()) for _ in range(len(boxes.values()))]]
        
        # for color, agents_by_color in self.agents_by_color.items():
        #     boxes = self.boxes_by_color.get(color, [])

        #     for agent in agents_by_color:
        #         if(boxes):
        #             for box in boxes:
        #                 goal_uid = HTNHelper.get_closest_goal_uid_to_box(box)
                        

        return reachability_matrix
    
    def has_any_task_left(self, current_state):
        for agent in current_state.agents:
            goal_uid = HTNHelper.get_closest_goal_uid_to_agent(agent)
            if(goal_uid != None):
                if State.goal_map[goal_uid][agent.pos.y][agent.pos.x] != 0:
                    return True
        for _, boxes in self.boxes_by_color.items():
            if len(boxes) > 0:
                return True

        return False
