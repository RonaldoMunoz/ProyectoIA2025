from agent.SearchAlgorithm import SearchAlgorithm
import heapq
from typing import Tuple
class AStar(SearchAlgorithm):
    def heuristic(self, a, b):
        # Manhattan
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))

    def find_path(self, start, goal) -> (list, bool):
        open_list = []
        heapq.heappush(open_list, (self.heuristic(start, goal), start))
        g_cost = {start: 0}
        came_from = {}
        closed_set = set()

        moves = []                                # ← lista de pasos (aristas) válidos

        while open_list:
            f_current, current = heapq.heappop(open_list)
            if current in closed_set:
                continue
            closed_set.add(current)

            if current == goal:
                self.final_cost = g_cost[current]
                return self.reconstruct_path(came_from, current), True

            for neighbor in self.get_neighbors(current):
                cell = self.grid.get_cell(neighbor)
                tentative_g = g_cost[current] + cell.cost
                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g
                    f_cost = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_cost, neighbor))

        # No encontró la meta: devolvemos todos los pasos explorados y False
        return self.reconstruct_path(came_from, current), False
