class Conflict:
    # ai and aj are the two agents who have conflict at (pos, t)
    def __init__(self, object_i, object_j, pos, t):
        self.pos = pos
        self.object_i = object_i
        self.object_j = object_j
        self.t = t

    def __repr__(self):
        return f"Conflict(object_i={self.object_i}, object_j={self.object_j}, pos={self.pos}, t={self.t})"

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