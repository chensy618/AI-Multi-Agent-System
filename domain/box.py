import string
from domain.color import Color
from domain.position import Position


class Box:
    def __init__(self, pos: Position, value: string, uid: int, color: Color):
        self.pos = pos
        self.color = color
        self.value = value
        self.uid = uid

    def __repr__(self):
        return f"Box(pos={self.pos}, value={self.value}, uid={self.uid} color={self.color})"
    
    def getRealBoxId(self):
        if isinstance(self.id, int):
            return chr(self.id)
        else:
            return self.id