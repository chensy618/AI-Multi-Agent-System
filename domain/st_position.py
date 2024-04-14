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
    
    def __lt__(self, other):
        if isinstance(other, STPosition):
            return (self.x, self.y, self.t) < (other.x, other.y, other.t)
        return False
    
    def __le__(self, other):
        if isinstance(other, STPosition):
            return (self.x, self.y, self.t) <= (other.x, other.y, other.t)
        return False
    
    def __gt__(self, other):
        if isinstance(other, STPosition):
            return (self.x, self.y, self.t) > (other.x, other.y, other.t)
        return False
    
    def __ge__(self, other):
        if isinstance(other, STPosition):
            return (self.x, self.y, self.t) >= (other.x, other.y, other.t)
        return False
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def __hash__(self):
        return super().__hash__()
