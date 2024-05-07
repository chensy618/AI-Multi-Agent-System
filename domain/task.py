class Task:
    def __init__(self, box_uid: int, goal_uid: int):
        self.box_uid = box_uid
        self.goal_uid = goal_uid
        self.is_completed = False
        
    def task_complete(self):
        self.is_completed = True  # Mark the task as completed

    def __repr__(self):
        return f"Task(box_uid={self.box_uid}, goal_uid={self.goal_uid}, completed={self.is_completed})"