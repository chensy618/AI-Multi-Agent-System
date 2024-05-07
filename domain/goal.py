import string
from domain.position import Position


class Goal:
    def __init__(self, pos: Position, value: string, uid: int):
        self.uid = uid
        self.pos = pos
        self.value = value

    def __repr__(self):
        return f"Goal(uid={self.uid}, pos={self.pos}, value={self.value})"