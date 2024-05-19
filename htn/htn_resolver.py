from collections import deque
import sys

from domain.position import Position
from domain.task import SubTask, Task
from helper.distance_calc import DistanceCalc
from htn.htn_helper import HTNHelper
from htn.priority_resolver import PriorityResolver
from pathfinding.astar import astar
from state import State


class HTNResolver:
    completed_tasks = {}

    def __init__(self, initial_state: State):
        # Stores current assigned task for an agent
        self.round = {}

        #Stores the target task for an agent
        self.target = {}

        # Stores sub-tasks for an agent
        self.sub_task_round = {}

        # Target for an agent
        self.target = {}

        self.round_counter = 0

        self.sub_round_counter = 0

        agents = initial_state.agents
        boxes = initial_state.boxes

        # Stores list of Box for each Colour
        self.agents_by_color = HTNHelper.categorize_agents_by_color(agents)

        # Stores list of Agent for each Colour
        self.boxes_by_color = HTNHelper.categorize_boxes_by_color(boxes)

        self.priority_resolver = PriorityResolver()

        self.reachability_matrix = {}

        self.round_planned_positions = {}

    def create_target(self, current_state):
        self.round_counter += 1

        boxes = current_state.boxes

        for color, agents_by_color in self.agents_by_color.items():
            boxes = self.boxes_by_color.get(color, [])

            for agent in agents_by_color:
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
                    if (goal_uid is None):
                        continue
                    self.target[agent.value] = Task(min_box.uid, min_box.value, goal_uid)
                else:
                    goal_uid = HTNHelper.get_closest_goal_uid_to_agent(agent)
                    if goal_uid is None:
                        continue
                    self.round[agent.value] = Task(-1, None, goal_uid)

    def has_any_subtask_left(self):
        return len(self.target.keys()) > 0

    def create_subtask(self, current_state: State):
        for agent_value, target_task in list(self.target.items()):
                agent = current_state.get_agent_by_uid(agent_value)
                if target_task.box_uid is not None:
                    if(self.sub_task_round.get(agent_value) and len(self.sub_task_round[agent_value]) > 0):
                        self.round[agent_value] = self.sub_task_round[agent_value].pop(0)
                        self.sub_round_counter += 1
                        return 
                    else:
                        print("Here we fking are", file=sys.stderr)
                        box = current_state.boxes[target_task.box_uid]
                        self.round[agent_value] = self.target[agent_value]
                        print("box",box, file=sys.stderr)
                        print("boxes_by_color[agent.color]", self.boxes_by_color[agent.color], file=sys.stderr)
                        self.boxes_by_color[agent.color].remove(box)
                        del self.target[agent_value]

                else:
                    raise RuntimeError("box_uid cannot be None, check self.target item assignment")

    def create_sub_round(self, current_state: State):
        print("\n\n===========SUBROUND===========", file=sys.stderr)

        self.initialize_reachability(current_state)

        for agent_uid, reachability_matrix in self.reachability_matrix.items():
            task_order_box_uids = self.priority_resolver.find_task_order(reachability_matrix, self.target[agent_uid].box_uid)

            subtask_boxes = [current_state.boxes[box_uid] for box_uid in task_order_box_uids if self.target[agent_uid].box_uid != box_uid]
            
            # TODO: It only works for one agent now 
            avoid_positions = self.round_planned_positions[agent_uid]
            for box in subtask_boxes:
                new_temp_goal_pos = self.priority_resolver.find_first_free_neighbour(current_state, box.pos, avoid_positions)
                avoid_positions.append(new_temp_goal_pos)
                new_subtask = SubTask(box.uid, box.value, new_temp_goal_pos)

                if self.sub_task_round.get(agent_uid) is None:
                    self.sub_task_round[agent_uid] = []

                self.sub_task_round[agent_uid].append(new_subtask)

            print("SUB ROUND ->", self.sub_task_round, file=sys.stderr)

        print("===========SUBROUND===========\n\n", file=sys.stderr)

    def initialize_reachability(self, initial_state: State):
        boxes = initial_state.boxes


        for agent_uid, task in self.target.items():
            state_from_agent_perspective = initial_state.from_agent_perspective_min(agent_uid, self.target) 
            
            plan = astar(initial_state, task)

            if(plan):
                continue
            else:
                # Check if our plan is blocked
                plan_from_agent_state = astar(state_from_agent_perspective, task)


                if(not plan_from_agent_state):
                    raise RuntimeError("Problem is not solvable, or task was assigned incorrectly")
                
                size = len(boxes.values())
                self.reachability_matrix[agent_uid] = [[False] * size for _ in range(size)]
                self.round_planned_positions[agent_uid] = HTNHelper.get_list_positions_from_actions(plan_from_agent_state, state_from_agent_perspective.agents[0].pos)
                
                for box in initial_state.boxes.values():
                    if(box.uid != task.box_uid and box.pos in self.round_planned_positions[agent_uid]):
                        self.reachability_matrix[agent_uid][task.box_uid][box.uid] = True
    
    def has_any_task_left(self, current_state):
        for goal in current_state.goals:
            if not current_state.is_goal_achieved(goal):
                return True

        return False
