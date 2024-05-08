import string


class Task:
    def __init__(self, box_uid: int, box_value: string, goal_uid: int):
        self.box_uid = box_uid
        self.goal_uid = goal_uid
        self.box_value = box_value
        
    def __repr__(self):
        return f"Task(box_uid={self.box_uid}, box_value={self.box_value}, goal_uid={self.goal_uid})"