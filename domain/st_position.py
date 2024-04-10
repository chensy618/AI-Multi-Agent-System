from domain.position import Position

# Position with time included as well, might be used for space-time-a*
class STPosition(Position):
    def __init__(self, x, y, t):
        super().__init__(x, y)
        self.t = t

    def __repr__(self):
        return f"STPosition(x={self.x}, y={self.y}, t={self.t})"

    def __eq__(self, other):
        if isinstance(other, STPosition):
            return self.x == other.x and self.y == other.y and self.t == other.t
        return False