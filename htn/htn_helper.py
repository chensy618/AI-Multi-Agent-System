class HTNHelper:
    @staticmethod
    def categorize_agents_by_color(agents):
        agents_by_color = {}
        for agent in agents:
            if agent.color not in agents_by_color:
                agents_by_color[agent.color] = []
            agents_by_color[agent.color].append(agent)
        return agents_by_color
    
    @staticmethod
    def categorize_boxes_by_color(boxes):
        boxes_by_color = {}
        for box in boxes:
            if box.color not in boxes_by_color:
                boxes_by_color[box.color] = []
            boxes_by_color[box.color].append(box)
        return boxes_by_color