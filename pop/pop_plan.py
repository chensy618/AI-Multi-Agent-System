class POPPlan:
    def __init__(self):
        self.action = []    # list of actions in the plan
        self.orderings = [] #  List of tuples (action_a, action_b) indicating action_a must come before action_b
        self.causal_links = [] # List of tuples (action_a, condition, action_b) indicating action_a achieves a condition for action_b

    def add_action(self, action):
        self.action.append(action)
    
    def add_ordering(self, action_a, action_b):
        self.orderings.append((action_a, action_b))

    def add_causal_link(self, action_a, condition, action_b):
        self.causal_links.append((action_a, condition, action_b))
    
    def is_consistent(self):
        # Check if the plan is consistent
        return True
    