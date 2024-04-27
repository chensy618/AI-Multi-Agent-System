import string
from domain.color import Color
from domain.position import Position


class Box:
    def __init__(self, pos: Position, id: string, color: Color):
        self.pos = pos
        self.color = color
        self.id = id

    def __repr__(self):
        return f"Box(pos={self.pos}, id={self.id}, color={self.color})"
    
    def getRealBoxId(self):
        return chr(self.id)