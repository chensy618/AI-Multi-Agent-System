from enum import Enum, unique

from domain.position import Position

class Direction:
    north = Position(0, -1)
    south = Position(0, 1)
    east = Position(1, 0)
    west = Position(-1, 0)

@unique
class ActionType(Enum):
    NoOp = 0
    Move = 1
    Push = 2
    Pull = 3

@unique
class Action(Enum):

    NoOp = ("NoOp", ActionType.NoOp, Position(0, 0), Position(0, 0))

    MoveN = ("Move(N)", ActionType.Move, Direction.north, Position(0, 0))
    MoveS = ("Move(S)", ActionType.Move, Direction.south, Position(0, 0))
    MoveE = ("Move(E)", ActionType.Move, Direction.east,  Position(0, 0))
    MoveW = ("Move(W)", ActionType.Move, Direction.west,  Position(0, 0))
    # Push actions
    PushN  = ("Push(N,N)", ActionType.Push, Direction.north, Direction.north)
    PushNE = ("Push(N,E)", ActionType.Push, Direction.north, Direction.east)
    PushNW = ("Push(N,W)", ActionType.Push, Direction.north, Direction.west)
    PushS  = ("Push(S,S)", ActionType.Push, Direction.south, Direction.south)
    PushSE = ("Push(S,E)", ActionType.Push, Direction.south, Direction.east)
    PushSW = ("Push(S,W)", ActionType.Push, Direction.south, Direction.west)
    PushE  = ("Push(E,E)", ActionType.Push, Direction.east,  Direction.east)
    PushEN = ("Push(E,N)", ActionType.Push, Direction.east,  Direction.north)
    PushES = ("Push(E,S)", ActionType.Push, Direction.east,  Direction.south)
    PushW  = ("Push(W,W)", ActionType.Push, Direction.west,  Direction.west)
    PushWN = ("Push(W,N)", ActionType.Push, Direction.west,  Direction.north)
    PushWS = ("Push(W,S)", ActionType.Push, Direction.west,  Direction.south)

    # Pull actions
    PullN  = ("Pull(N,N)", ActionType.Pull, Direction.north, Direction.north)
    PullNE = ("Pull(N,E)", ActionType.Pull, Direction.north, Direction.east)
    PullNW = ("Pull(N,W)", ActionType.Pull, Direction.north, Direction.west)
    PullS  = ("Pull(S,S)", ActionType.Pull, Direction.south, Direction.south)
    PullSE = ("Pull(S,E)", ActionType.Pull, Direction.south, Direction.east)
    PullSW = ("Pull(S,W)", ActionType.Pull, Direction.south, Direction.west)
    PullE  = ("Pull(E,E)", ActionType.Pull, Direction.east,  Direction.east)
    PullEN = ("Pull(E,N)", ActionType.Pull, Direction.east,  Direction.north)
    PullES = ("Pull(E,S)", ActionType.Pull, Direction.east,  Direction.south)
    PullW  = ("Pull(W,W)", ActionType.Pull, Direction.west,  Direction.west)
    PullWN = ("Pull(W,N)", ActionType.Pull, Direction.west,  Direction.north)
    PullWS = ("Pull(W,S)", ActionType.Pull, Direction.west,  Direction.south)
    
    def get_position(self) -> Position:
        return self.agent_rel_pos

    def __init__(self, name, type, agent_rel_pos, box_rel_pos):
        self.name_ = name
        self.type = type
        self.agent_rel_pos = agent_rel_pos # displacement of agent by position
        self.box_rel_pos = box_rel_pos # displacement of box by position

    def __repr__(self):
        return self.name
