class Conflict:
    # ai and aj are the two entity who have conflict at (pos, t)
    # we have box-box conflict, agent-agent conflict, agent-box conflict
    def __init__(self, conflict_type, ai, aj, pos, t):
        self.conflict_type = conflict_type
        self.pos = pos
        self.ai = ai
        self.aj = aj
        self.t = t

    def __repr__(self):
        return f"Conflict(conflict_type = {self.conflict_type}, ai={self.ai}, aj={self.aj}, pos={self.pos}, t={self.t})"