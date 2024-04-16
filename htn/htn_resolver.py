from collections import deque

from astar import astar
from domain.agent import Agent
from domain.box import Box
from domain.goal import Goal
from htn.htn_helper import HTNHelper
from state import State


class HTNResolver:
    # Stores list of (agentId, boxId) tuples for each agent
    agent_tasks = {}
    
    # Stores current assigned task for an agent
    assigned_tasks = {}
    
    @staticmethod
    def initialize_problems(initial_state: State):
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
                if agent.id not in HTNResolver.agent_tasks:
                    HTNResolver.agent_tasks[agent.id] = deque()

                HTNResolver.agent_tasks[agent.id].append((agent.id, box.id))

    @staticmethod
    def set_assigned_taskss(relaxed_state: State):
        # TODO Get closest box to agent for assigning
        for agent_id in list(HTNResolver.agent_tasks.keys()):
            if(HTNResolver.assigned_tasks.get(agent_id) is None and HTNResolver.agent_tasks[agent_id]):
                task = HTNResolver.agent_tasks[agent_id].popleft()
                # TODO Modify current state for the single agent task
                HTNResolver.assigned_tasks[agent_id] = astar(relaxed_state, []) 

    @staticmethod        
    def has_any_task_left():
        for _, tasks in HTNResolver.agent_tasks:
            if len(tasks) > 0: return True

        return False
