from domain.position import Position
from state import State


class PriorityResolver:
    def find_task_order(self, reachability_matrix, target_box, task_order=None, visited=None):
        if task_order is None:
            task_order = []
        if visited is None:
            visited = set()

        # Add the current target_box to the visited set to detect cycles (deadlocks)
        if target_box in visited:
            raise ValueError(f"Deadlock detected when trying to move box {chr(target_box + ord('A'))}")
        visited.add(target_box)

        # Recursive case: resolve dependencies first
        for dependency, can_reach in enumerate(reachability_matrix[target_box]):
            if can_reach:
                if dependency not in task_order:  # Avoid duplicate entries
                    self.find_task_order(reachability_matrix, dependency, task_order, visited)

        # After resolving dependencies, add the current target_box to the task order
        if target_box not in task_order:
            task_order.append(target_box)

        # Remove the current target_box from visited to allow different paths to explore it independently
        visited.remove(target_box)

        return task_order
    
    def find_first_free_neighbour(self, state: State, pos, visited=None):

        if visited is None:
            visited = set()
        
        # Check if the current position is within bounds and not visited
        if (pos.y, pos.x) in visited or pos.y < 0 or pos.y >= len(state.walls) or pos.x < 0 or pos.x >= len(state.walls[0]):
            return None

        # Mark the current cell as visited
        visited.add(pos)
        
        # Check if the current cell is free
        if state.walls[pos.y][pos.x] == False:
            return pos

        # Explore the four possible neighboring cells (up, down, left, right)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for dy, dx in directions:
            new_pos = Position(pos.y + dy, pos.x + dx)
            result = self.find_first_free_neighbour(state, pos, visited)
            if result:
                return result

        # If no free cell is found, return None
        return None