from enum import Enum, unique

from domain.position import Position

@unique
class ActionType(Enum):
    NoOp = 0
    Move = 1
    Push = 2
    Pull = 3

@unique
class Action(Enum):
    NoOp = ("NoOp", ActionType.NoOp, 0, 0, 0, 0)

    MoveN = ("Move(N)", ActionType.Move, Position(-1, 0), 0, 0)
    MoveS = ("Move(S)", ActionType.Move, Position(1, 0), 0, 0)
    MoveE = ("Move(E)", ActionType.Move, Position(0, 1), 0, 0)
    MoveW = ("Move(W)", ActionType.Move, Position(0, -1), 0, 0)
    
    def __init__(self, name, type, aRPos, bRPos):
        self.name_ = name
        self.type = type
        self.aRPos = aRPos # displacement of agent by position
        self.bRPos = bRPos # displacement of box by position
