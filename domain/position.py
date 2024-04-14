class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Position(x={self.x}, y={self.y})"
    
    def __add__(self, other):
        if isinstance(other, Position):
            return Position(self.x + other.x, self.y + other.y)
        else:
            raise ValueError("Cannot add Position with type {}".format(type(other)))
        
    def __eq__(self, other):
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y
        return False
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __sub__(self, other):
        if isinstance(other, Position):
            return Position(self.x - other.x, self.y - other.y)
    
    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)
    
    def __le__(self, other):
        return (self.x, self.y) <= (other.x, other.y)
    
    def __gt__(self, other):
        return (self.x, self.y) > (other.x, other.y)
    
    def __ge__(self, other):
        return (self.x, self.y) >= (other.x, other.y)
    
    def __ne__(self, other):
        return not self.__eq__(other)   
        

