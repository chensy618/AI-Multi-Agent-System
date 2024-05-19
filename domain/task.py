import string

from domain.position import Position


class Task:
    def __init__(self, box_uid: int, box_value: string, goal_uid: int):
        self.box_uid = box_uid
        self.goal_uid = goal_uid
        self.box_value = box_value
        
    def __repr__(self):
        return f"Task(box_uid={self.box_uid}, box_value={self.box_value}, goal_uid={self.goal_uid})"
    

class SubTask(Task):
    def __init__(self, box_uid: int, box_value: string, goal_pos: Position):
        self.box_uid = box_uid
        self.goal_pos = goal_pos
        self.box_value = box_value
        
    def __repr__(self):
        return f"Task(box_uid={self.box_uid}, box_value={self.box_value}, goal_pos={self.goal_pos})"
    