class Box:
    def __init__(self, pos, id, color):
        self.pos = pos
        self.color = color
        self.id = id # stores int box id (not char)

    def __repr__(self):
        return f"Box(pos={self.pos}, id={self.id}, color={self.color})"
    
    def getRealBoxId(self):
        return chr(self.id)