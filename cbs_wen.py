from astar import astar

def conflict_based_search(problem_list):
    agents = [agent.id for state in problem_list for agent in state.agents]
    print(f"---agents--{agents}")
    root = Node()
    root.constraints = []
    for problem in problem_list:
        root.solution[problem.agent] = astar(problem)
    # root.cost = cost(root.solution)
    # frontier = PriorityQueue(root, lambda n: n.cost)
    # while not frontier.empty:
    #     node = frontier.get()
    #     if is_valid(node.solution):
    #         return node.solution
    #     conflict = find_first_conflict(node.solution)
    #     for problem in problem_list:
    #         if problem.agent in [conflict.ai, conflict.aj]:
    #             m = node.copy()
    #             m.constraints.append(Constraint(problem.agent, STPosition(conflict.pos, conflict.t)))
    #             m.solution[problem.agent] = space_time_a_star(problem, m.constraints)
    #             m.cost = cost(m.solution)

    #             if(m.cost < sys.maxint):
    #                 frontier.put(m)
