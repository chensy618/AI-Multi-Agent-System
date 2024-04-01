class Node:
    # A 'state' in the CT tree
    def __init__(self, root, constraints, solution, cost):
        self.root = root
        self.constraints = constraints
        self.solution = solution
        self.cost = cost

    def __repr__(self):
        return f"Node(root={self.root}, solution={self.solution}, cost={self.cost})"