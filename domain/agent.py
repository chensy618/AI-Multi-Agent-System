from domain.color import Color
from domain.position import Position

class Agent:
    def __init__(self, pos: Position, value: int, color: Color):
        self.pos = pos
        self.color = color
        self.value = value

    def __repr__(self):
        return f"Agent(value={self.value}, pos={self.pos}, color={self.color})"
    
    def move(self, dx, dy):
        self.pos = Position(self.pos.x + dx, self.pos.y + dy)
