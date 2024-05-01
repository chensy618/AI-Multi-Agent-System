class Task:
    def __init__(self, agent_uid: int, box_uid: int, goal_uid: int):
        self.agent_uid = agent_uid
        self.box_uid = box_uid
        self.goal_uid = goal_uid

    def __repr__(self):
        return f"Task(agent_uid={self.agent_uid}, box_uid={self.box_uid} goal_uid={self.goal_uid})"