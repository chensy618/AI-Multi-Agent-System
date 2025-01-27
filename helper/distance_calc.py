from state import State
import sys

class DistanceCalc:
    @staticmethod
    def manhatten_distance(pos1, pos2) -> 'int':
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)
    staticmethod
    def euclidian_distance(pos1, pos2) -> 'int':
        return ((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)**0.5
    staticmethod
    def chebyshev_distance(pos1, pos2) -> 'int':
        return max(abs(pos1.x - pos2.x), abs(pos1.y - pos2.y))
    staticmethod
    def pos_to_box_distance(box, pos) -> 'int':
        # Check if the box has already moved from its initial position
        if(State.box_goal_map[box.uid][box.pos.y][box.pos.x] != 0):
            return DistanceCalc.manhatten_distance(pos, box.pos)
        else:
            return State.box_goal_map[box.uid][pos.y][pos.x]
        
    def calculate_box_task(agent_box, agent, goal_uid) -> 'int':
        if(goal_uid is None): return sys.maxsize
        agent_to_box_dist = DistanceCalc.pos_to_box_distance(agent_box, agent.pos)
        box_to_goal_dist = State.goal_map[goal_uid][agent_box.pos.y][agent_box.pos.x]
        if agent_to_box_dist == None:
            return box_to_goal_dist
        elif box_to_goal_dist == None:
            return agent_to_box_dist

        return agent_to_box_dist + box_to_goal_dist
    
    def calculate_agent_task(agent, goal_uid) -> 'int':
        return State.goal_map[goal_uid][agent.pos.y][agent.pos.x]
