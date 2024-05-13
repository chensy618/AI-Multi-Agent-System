import string
from domain.color import Color
from domain.position import Position


class Box:
    def __init__(self, pos: Position, value: string, uid: int, color: Color):
        self.pos = pos
        self.color = color
        self.value = value
        self.uid = uid
        self.dist = 0

    def __repr__(self):
        return f"Box(pos={self.pos}, value={self.value}, uid={self.uid} color={self.color})"
    
    def getRealBoxId(self):
        if isinstance(self.uid, int):
            return chr(self.uid)
        else:
            return self.uid