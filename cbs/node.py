import copy

from domain.constraint import Constraint


class Node:
    def __init__(self):
        self.solution = {}  # Maps agent IDs to their paths
        self.constraints = []  # List of constraints on the agents
        self.cost = 0  # Cost of the solution

    def copy(self):
        # Create a deep copy of the node
        new_node = Node()
        new_node.solution = copy.deepcopy(self.solution)
        new_node.constraints = copy.deepcopy(self.constraints)
        # Note: Cost is a numeric value, so a shallow copy is fine
        new_node.cost = self.cost
        return new_node
