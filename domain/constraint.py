class Constraint:
    # It's a constraint on one of the agent at (pos, t)
    def __init__(self, agent, pos, t):
        self.pos = pos
        self.agent = agent
        self.t = t

    def __repr__(self):
        return f"Constraint(agent={self.agent}, pos={self.pos}, t={self.t})"