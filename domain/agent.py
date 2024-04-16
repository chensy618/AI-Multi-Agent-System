from domain.position import Position
class Agent:
    def __init__(self, pos, id, color):
        self.pos = pos
        self.color = color
        self.id = id

    def __repr__(self):
        return f"Agent(pos={self.pos}, id={self.id}, color={self.color})"
    
    def move(self, dx, dy):
        self.pos = Position(self.pos.x + dx, self.pos.y + dy)
