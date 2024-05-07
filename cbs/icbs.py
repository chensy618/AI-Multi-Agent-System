# from queue import PriorityQueue
# import sys
# import numpy as np
# from cbs.cbs import find_first_conflict, is_valid, space_time_a_star
# from cbs.meta_agent import MetaAgent
# from cbs.node import Node
# from domain.agent import Agent
# from domain.conflict import Conflict
# from domain.constraint import Constraint
# from domain.st_position import STPosition

# class ICBSSolver:
#     def __init__(self, numOfAgents: int):
#         self.matrix = np.zeros(numOfAgents, numOfAgents)
#         self.max_conflict = 69

#     def find_solution(self, problem, agents, cost):
#         root = Node()
#         root.constraints = []
#         for agent in agents:
#             root.solution[agent] = space_time_a_star(problem[agent], [])
#         root.cost = cost(root.solution)
#         open = PriorityQueue(root, lambda n: n.cost)
#         while not open.empty:
#             node = open.get()
#             if is_valid(node.solution):
#                 return node.solution
#             conflict = find_first_conflict(node.solution)

#             if(self.should_merge(conflict.ai, conflict.aj)): 
#                 meta_a = self.merge(conflict.ai, conflict.aj, node.constraints)
#                 node.constraints = meta_a.constraints
#                 node.solution = space_time_a_star_global(problem, []) # TODO
#                 open.put(node)
#                 continue

#             for agent in [conflict.ai, conflict.aj]:
#                 m = node.copy()
#                 m.constraints.append(Constraint(agent, STPosition(conflict.pos, conflict.t)))
#                 m.solution[agent] = space_time_a_star(problem[agent], m.constraints)
#                 m.cost = cost(m.solution)

#                 if(m.cost < sys.maxint):
#                     open.put(m)

#     def should_merge(self, ai: Agent, aj: Agent) -> bool:
#         assert ai.id != aj.id, "-- Should be no conflict between the same agent --"

#         self.matrix[ai.id][aj.id] += 1
#         self.matrix[aj.id][ai.id] += 1

#         if(self.matrix[ai.id][aj.id] > self.max_conflict):
#             return True
#         return False

#     def merge(self, ai: Agent, aj: Agent, constraints: list[Constraint]) -> MetaAgent:
#         new_meta_a = MetaAgent()

#         new_meta_a.agents.append(ai)
#         new_meta_a.agents.append(aj)

#         external_constraints = [c for c in constraints if c.agentId != ai.id and c.agentId != aj.id]

#         new_meta_a.constraints.extend(external_constraints)
#         return new_meta_a
    
#     def space_time_a_star_global(self):
#         return