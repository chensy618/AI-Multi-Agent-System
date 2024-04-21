class Conflict:
    # ai and aj are the two agents who have conflict at (pos, t)
    def __init__(self, ai, aj, pos, t):
        self.pos = pos
        self.ai = ai
        self.aj = aj
        self.t = t

    def __repr__(self):
        return f"Conflict(ai={self.ai}, aj={self.aj}, pos={self.pos}, t={self.t})"
