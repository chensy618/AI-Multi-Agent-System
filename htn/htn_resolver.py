from collections import deque
import sys

from htn.htn_helper import HTNHelper
from state import State


class HTNResolver:
    def __init__(self):
        # Stores list of (agentId, boxId) tuples for each agent
        self.agent_tasks = {}
        
        # Stores current assigned task for an agent
        self.round = {}
        
        self.round_counter = 0

    def initialize_problems(self, initial_state: State):
        agents = initial_state.agents
        boxes = initial_state.boxes

        agents_by_color = HTNHelper.categorize_agents_by_color(agents)
        boxes_by_color = HTNHelper.categorize_boxes_by_color(boxes)
        for color, color_boxes in boxes_by_color.items():
            # Retrieve the agents of the current box color
            color_agents = agents_by_color.get(color, [])

            # If there are no agents of this color, skip to the next color
            if not color_agents:
                continue  

            # Distribute boxes equally among agents of the same color
            for i, box in enumerate(color_boxes):
                # Determine which agent should receive this box based on a round-robin distribution
                # TODO Assign box to agent based on distance
                agent_index = i % len(color_agents)
                agent = color_agents[agent_index]

                # TODO Check if generating plan for agent -> box, is possible (not blocked)

                # Create agent_tasks for each agent
                if agent.uid not in self.agent_tasks:
                    self.agent_tasks[agent.uid] = deque()

                self.agent_tasks[agent.uid].append(box)

    def create_round(self):
        self.round_counter += 1

        for agent_id in list(self.agent_tasks.keys()):
            if(self.round.get(agent_id) is not None):
                raise RuntimeError(f"Agent {agent_id} has not completed the previous task")
            box = self.agent_tasks[agent_id].popleft()
            goal_uid = HTNHelper.get_closest_goal_uid_to_box(box)
            self.round[agent_id] = (box.uid, goal_uid) 

    def create_plans(self, current_state: State):
        plans = {}
        for agent_id in list(self.round.keys()):
            (box_uid, goal_uid) = self.round.pop(agent_id)  # Remove the agent's task from self.round
            relaxed_state = current_state.from_agent_perspective(agent_id)
            # plan = astar(relaxed_state)
            # plans[agent_id] = plan
        return plans
        
    def has_any_task_left(self):
        for _, tasks in self.agent_tasks.items():
            if len(tasks) > 0: return True

        return False
