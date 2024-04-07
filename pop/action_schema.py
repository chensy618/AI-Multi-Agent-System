class POPAction:
    def __init__(self, name, precondition, effects):
        self.name = name
        self.precondition = precondition
        self.effects = effects

class POPState:
    def __init__(self, state, start_effects, goal_preconditions):
        self.initiate_state = POPAction("Start", [], start_effects)
        self.goal_state = POPAction("Goal", goal_preconditions, [])
        self.state = state

