from domain.st_position import STPosition


class Constraint:
    # It's a constraint on one of the agent at (pos, t)
    def __init__(self, agentId: int, st_pos: STPosition):
        self.agentId = agentId
        self.st_pos = st_pos

    def __repr__(self):
        return f"Constraint(agentId={self.agentId}, st_pos={self.st_pos})"