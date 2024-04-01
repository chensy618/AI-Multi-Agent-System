from queue import PriorityQueue
import sys
from cbs.node import Node
from domain.conflict import Conflict
from domain.constraint import Constraint
from domain.st_position import STPosition

def conflict_based_search(problem, agents, cost):
    root = Node()
    root.constraints = []
    for agent in agents:
        root.solution[agent] = space_time_a_star(problem[agent], [])
    root.cost = cost(root.solution)
    frontier = PriorityQueue(root, lambda n: n.cost)
    while not frontier.empty:
        node = frontier.get()
        if is_valid(node.solution):
            return node.solution
        conflict = find_first_conflict(node.solution)
        for agent in [conflict.ai, conflict.aj]:
            m = node.copy()
            m.constraints.append(Constraint(agent, STPosition(conflict.pos, conflict.t)))
            m.solution[agent] = space_time_a_star(problem[agent], m.constraints)
            m.cost = cost(m.solution)

            if(m.cost < sys.maxint):
                frontier.put(m)

def space_time_a_star(sub_problem, conflicts):
    path = []
    return path

def cost_soc(plans):
    sumOfCost = 0
    for p in plans:
        sumOfCost += len(p)
    return sumOfCost

def cost_ms(plans):
    makeSpan = 0
    for p in plans:
        if (len(p) > makeSpan):
            makeSpan = len(p)
    return makeSpan

def is_valid(solution):
    return find_first_conflict(solution) is None

def find_first_conflict(solution):
    visited = set()
    for path in solution:
        for space_time_position in path:
            if(space_time_position in visited):
                return space_time_position
            visited.add(space_time_position)
    return None