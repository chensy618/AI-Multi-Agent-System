class Conflict:
    # ai and aj are the two agents who have conflict at (pos, t)
    def __init__(self, ai, aj, pos, t):
        self.pos = pos
        self.ai = ai
        self.aj = aj
        self.t = t

    def __repr__(self):
        return f"Conflict(ai={self.ai}, aj={self.aj}, pos={self.pos}, t={self.t})"
    
    def __eq__(self, other):
        return self.ai == other.ai and self.aj == other.aj and self.pos == other.pos and self.t == other.t
    
    def __hash__(self):
        return hash((self.ai, self.aj, self.pos, self.t))
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
        return f"MoveAwayConflict(ai={self.ai}, aj={self.aj}, current_pos={self.current_pos} ,avoid_pos_list={self.avoid_pos_list}, t={self.t})"
    
    def __eq__(self, other):
        return self.ai == other.ai and self.aj == other.aj and self.current_pos == other.current_pos and self.t == other.t
    
    def __hash__(self):
        return hash((self.ai, self.aj, self.current_pos, self.t))
    
class FollowConflict:
    # ai is following another agent at the timestep t
    def __init__(self, ai, t):
        self.ai = ai
        self.t = t

    def __repr__(self):
        return f"FollowConflict(ai={self.ai}, t={self.t})"
    
    def __eq__(self, other):
        return self.ai == other.ai and self.t == other.t
    
    def __hash__(self):
        return hash((self.ai, self.t))
    


class MetaAgentConflict:
    # ai and aj has a lot of conflicts and can't be solved
    def __init__(self, ai, aj):
        self.ai = ai
        self.aj = aj

    def __repr__(self):
        return f"MetaAgentConflict(ai={self.ai}, aj={self.aj})"
    
    def __eq__(self, other):
        return self.ai == other.ai and self.aj == other.aj
    
    def __hash__(self):
        return hash((self.ai, self.aj))
