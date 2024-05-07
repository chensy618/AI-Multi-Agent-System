class Conflict:
    # ai and aj are the two agents who have conflict at (pos, t)
    def __init__(self, ai, aj, pos, t):
        self.pos = pos
        self.ai = ai
        self.aj = aj
        self.t = t

    def __repr__(self):
        return f"Conflict(ai={self.ai}, aj={self.aj}, pos={self.pos}, t={self.t})"

class MoveAwayConflict:
    # ai has to move away from the position list
    # aj was blocked
    def __init__(self, ai, aj, current_pos, avoid_pos_list, t):
        self.avoid_pos_list = avoid_pos_list
        self.aj = aj
        self.ai = ai
        self.current_pos = current_pos
        self.t = t

    def __repr__(self):
        return f"Conflict(ai={self.ai}, aj={self.aj}, current_pos={self.current_pos} ,avoid_pos_list={self.avoid_pos_list}, t={self.t})"
    
class FollowConflict:
    # ai is following another agent at the timestep t
    def __init__(self, ai, t):
        self.ai = ai
        self.t = t

    def __repr__(self):
        return f"Conflict(ai={self.ai}, t={self.t})"
