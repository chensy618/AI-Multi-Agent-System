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
    NoOp = ("NoOp", ActionType.NoOp, Position(0,0), Position(0,0))

    MoveN = ("Move(N)", ActionType.Move, Position(-1, 0), Position(0, 0))
    MoveS = ("Move(S)", ActionType.Move, Position(1, 0),  Position(0, 0))
    MoveE = ("Move(E)", ActionType.Move, Position(0, 1),  Position(0, 0))
    MoveW = ("Move(W)", ActionType.Move, Position(0, -1), Position(0, 0))
    # Push actions
    PushN  = ("Push(N,N)", ActionType.Push, Position(-1, 0), Position(-1, 0))
    PushNE = ("Push(N,E)", ActionType.Push, Position(-1, 0), Position(0, 1))
    PushNW = ("Push(N,W)", ActionType.Push, Position(-1, 0), Position(0, -1))
    PushS  = ("Push(S,S)", ActionType.Push, Position(1, 0),  Position(1, 0))
    PushSE = ("Push(S,E)", ActionType.Push, Position(1, 0),  Position(0, 1))
    PushSW = ("Push(S,W)", ActionType.Push, Position(1, 0),  Position(0, -1))
    PushE  = ("Push(E,E)", ActionType.Push, Position(0, 1),  Position(0, 1))
    PushEN = ("Push(E,N)", ActionType.Push, Position(0, 1),  Position(-1, 0))
    PushES = ("Push(E,S)", ActionType.Push, Position(0, 1),  Position(1, 0))
    PushW  = ("Push(W,W)", ActionType.Push, Position(0, -1), Position(0, -1))
    PushWN = ("Push(W,N)", ActionType.Push, Position(0, -1), Position(-1, 0))
    PushWS = ("Push(W,S)", ActionType.Push, Position(0, -1), Position(1, 0))

    # Pull actions
    PullN  = ("Pull(N,N)", ActionType.Pull, Position(-1, 0), Position(-1, 0))
    PullNE = ("Pull(N,E)", ActionType.Pull, Position(-1, 0), Position(0, 1))
    PullNW = ("Pull(N,W)", ActionType.Pull, Position(-1, 0), Position(0, -1))
    PullS  = ("Pull(S,S)", ActionType.Pull, Position(1, 0),  Position(1, 0))
    PullSE = ("Pull(S,E)", ActionType.Pull, Position(1, 0),  Position(0, 1))
    PullSW = ("Pull(S,W)", ActionType.Pull, Position(1, 0),  Position(0, -1))
    PullE  = ("Pull(E,E)", ActionType.Pull, Position(0, 1),  Position(0, 1))
    PullEN = ("Pull(E,N)", ActionType.Pull, Position(0, 1),  Position(-1, 0))
    PullES = ("Pull(E,S)", ActionType.Pull, Position(0, 1),  Position(1, 0))
    PullW  = ("Pull(W,W)", ActionType.Pull, Position(0, -1), Position(0, -1))
    PullWN = ("Pull(W,N)", ActionType.Pull, Position(0, -1), Position(-1, 0))
    PullWS = ("Pull(W,S)", ActionType.Pull, Position(0, -1), Position(1, 0))
    
    def __init__(self, name, type, agent_rel_pos, box_rel_pos):
        self.name_ = name
        self.type = type
        self.agent_rel_pos = agent_rel_pos # displacement of agent by position
        self.box_rel_pos = box_rel_pos # displacement of box by position
