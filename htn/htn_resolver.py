from collections import deque
import sys

from domain.task import Task
from helper.distance_calc import DistanceCalc
from htn.htn_helper import HTNHelper
from state import State


class HTNResolver:
    completed_tasks = {}

    def __init__(self):
        # Stores current assigned task for an agent
        self.round = {}
        # Stores list of Box for each Colour
        self.boxes_by_color = {}

        # Stores list of Agent for each Colour
        self.agents_by_color = {}

        self.round_counter = 0

    def initialize_problems(self, initial_state: State):
        agents = initial_state.agents
        boxes = initial_state.boxes
        goals = initial_state.goals

        self.agents_by_color = HTNHelper.categorize_agents_by_color(agents)
        self.boxes_by_color = HTNHelper.categorize_boxes_by_color(initial_state, boxes, goals)

    def create_round(self, current_state):
        self.round_counter += 1

        boxes = current_state.boxes
        goals = current_state.goals
        self.boxes_by_color = HTNHelper.categorize_boxes_by_color(current_state, boxes, goals)

        for color, agents_by_color in self.agents_by_color.items():
            boxes = self.boxes_by_color.get(color, [])

            for agent in agents_by_color:
                # Check if we have box
                if boxes:
                    # min_box, goal_uid = min(
                    #     [
                    #         (
                    #             box, 
                    #             HTNHelper.get_closest_goal_uid_to_box(box)
                    #         ) 
                    #         for box in boxes
                    #     ],
                    #     key=lambda item: DistanceCalc.calculate_box_task(item[0], agent, item[1])
                    # )
                    goals = [goal for box in boxes for goal in State.goals if goal.value == box.value]
                    available_goals =  [goal for goal in goals if not current_state.is_goal_achieved(goal)]
                    box, goal = HTNHelper.prioritize_goal_box_by_difficulty(available_goals, boxes, agent)
                    #goal = HTNHelper.prioritize_goals_by_difficulty(available_goals, agent)
                    #box = HTNHelper.prioritize_boxes_by_difficulty(boxes, goal, agent)

                    print("priorities done", file=sys.stderr)
                    #self.boxes_by_color[agent.color].remove(box)
                    self.round[agent.value] = Task(box.uid, box.value, goal.uid)
                else:
                    print("without priorities", file=sys.stderr)
                    goal_uid = HTNHelper.get_closest_goal_uid_to_agent(agent)
                    self.round[agent.value] = Task(-1, None, goal_uid)
        
    def has_any_task_left(self, current_state):
        for agent in current_state.agents:
            goal_uid = HTNHelper.get_closest_goal_uid_to_agent(agent)
            if(goal_uid != None):
                if State.goal_map[goal_uid][agent.pos.y][agent.pos.x] != 0:
                    return True
        for goal in current_state.goals:
            if not current_state.is_goal_achieved(goal):
                return True
        #for _, boxes in self.boxes_by_color.items():
        #    if len(boxes) > 0:
        #        return True

        return False
