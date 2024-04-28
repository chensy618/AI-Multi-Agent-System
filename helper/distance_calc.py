from state import State


class DistanceCalc:
    @staticmethod
    def manhatten_distance(self, pos1, pos2) -> 'int':
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)
    
    def euclidian_distance(self, pos1, pos2) -> 'int':
        return ((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)**0.5
    
    def chebyshev_distance(self, pos1, pos2) -> 'int':
        return max(abs(pos1.x - pos2.x), abs(pos1.y - pos2.y))
    
    def pos_to_box_distance(box, pos) -> 'int':
        # Check if the box has already moved from its initial position
        if(State.box_goal_map[box.uid][box.pos.y][box.pos.x] != 0):
            return DistanceCalc.manhatten_distance(pos, box.pos)
        else:
            return State.box_goal_map[box.uid][pos.y][pos.x]