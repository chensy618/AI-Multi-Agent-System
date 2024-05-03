class Task:
    def __init__(self, box_uid: int, goal_uid: int):
        self.box_uid = box_uid
        self.goal_uid = goal_uid

    def __repr__(self):
        return f"Task(box_uid={self.box_uid} goal_uid={self.goal_uid})"