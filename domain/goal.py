import string
from domain.position import Position


class Goal:
    def __init__(self, pos: Position, value: string, uid: int, x1:int, y1:int, x2:int, y2:int):
        self.uid = uid
        self.pos = pos
        self.value = value
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __repr__(self):
        return f"Goal(uid={self.uid}, pos={self.pos}, value={self.value}, x1={self.x1}, y1={self.y1}, x2={self.x2}, y2={self.y2})"