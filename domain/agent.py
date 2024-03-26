class Agent:
    def __init__(self, pos, id, color):
        self.pos = pos
        self.color = color
        self.id = id

    def __repr__(self):
        return f"Agent(pos={self.pos}, id={self.id}, color={self.color})"