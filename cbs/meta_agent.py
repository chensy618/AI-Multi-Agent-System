from domain.agent import Agent
from domain.constraint import Constraint


class MetaAgent:
    def __init__(self):
        self.agents : list[Agent] = []
        self.constraints : list[Constraint] = []

    def __repr__(self):
        return f"MetaAgent(agents={self.agents}, constraints={self.constraints})"