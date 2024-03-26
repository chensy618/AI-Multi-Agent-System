class Goal:
    def __init__(self, pos, id):
        self.pos = pos
        self.id = id

    def __repr__(self):
        return f"Goal(pos={self.pos}, id={self.id})"