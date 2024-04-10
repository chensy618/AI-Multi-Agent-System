class Constraint:
    # It's a constraint on one of the agent at (pos, t)
    def __init__(self, agentId : int, pos, t):
        self.agentId = agentId
        self.pos = pos
        self.t = t

    def __repr__(self):
        return f"Constraint(agentId={self.agentId}, pos={self.pos}, t={self.t})"