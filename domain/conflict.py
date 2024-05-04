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
    def __init__(self, ai, current_pos, avoid_pos_list, t):
        self.avoid_pos_list = avoid_pos_list
        self.ai = ai
        self.current_pos = current_pos
        self.t = t

    def __repr__(self):
        return f"Conflict(ai={self.ai}, current_pos={self.current_pos} ,avoid_pos_list={self.avoid_pos_list}, t={self.t})"
