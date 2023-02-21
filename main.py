import heapq
import random
from enum import Enum
from typing import List


# The stack implementation
class Stack:
    def __init__(self) -> None:
        self.stack = []

    def is_empty(self):
        return len(self.stack) == 0

    def push(self, node):
        self.stack.append(node)

    def pop(self):
        if self.is_empty():
            return "Stack is empty"
        return self.stack.pop()

    def display(self):
        if self.is_empty():
            return "Stack is empty"
        return self.stack


class Node:
    def __init__(self, maze_node):
        self.maze_node = maze_node


# The maze implementation
class Cell(str, Enum):
    EMPTY = " "
    BARRIER = "X"
    START = "S"
    GOAL = "G"
    PATH = "*"
    ABSOLUTE_PATH = "#"


class Maze_Node:
    def __init__(self, y, x, heuristic: float = 0.0) -> None:
        self.y: int = y
        self.x: int = x
        self.heu_node = (heuristic, (self.y, self.x))
        self.node = (self.y, self.x)


class Maze:
    def __init__(self, columns, rows, barriers: List[Maze_Node], start: Maze_Node,
                 goal: Maze_Node) -> None:
        self._columns: int = columns
        self._rows: int = rows
        self.start: Maze_Node = start
        self.goal: Maze_Node = goal
        self.barriers: List[Maze_Node] = barriers
        # fill the maze with empty cells
        self._maze: List[List[Cell]] = [[Cell.EMPTY for c in range(columns)] for r in range(rows)]
        # add barriers to the maze
        for b in barriers:
            self._maze[b.y][b.x] = Cell.BARRIER
        # set the start node
        self._maze[start.y][start.x] = Cell.START
        # set the goal node
        self._maze[goal.y][goal.x] = Cell.GOAL

    def print_maze(self) -> str:
        output: str = ""
        for row in self._maze:
            output += " ".join([c.value for c in row]) + "\n"
        return output

    def mark_path(self, node: Maze_Node):
        self._maze[node.y][node.x] = Cell.PATH
        self._maze[self.start.y][self.start.x] = Cell.START
        self._maze[self.goal.y][self.goal.x] = Cell.GOAL

    def absolute_path(self, node: Maze_Node):
        self._maze[node.y][node.x] = Cell.ABSOLUTE_PATH

    # to clear the maze because after using we should be able to use the same maze with different barriers,
    # start and goal nodes
    def clear_path(self, maze):
        for node in maze:
            self._maze[node.y][node.x] = Cell.EMPTY

    @property
    def rows(self):
        return self._rows

    @property
    def maze(self):
        return self._maze

    @property
    def columns(self):
        return self._columns


# DFS implementation
class DFS:
    def __init__(self, maze: Maze):
        self.maze: Maze = maze
        # not visited nodes
        self.not_visited: Stack = Stack()
        self.all_maze_nodes = []
        # visited nodes
        self.visited = []
        self.time: int = 0

    def dfs_algorithm(self):
        initial_state = self.maze.start.node
        goal_state = self.maze.goal.node

        self.not_visited.push(initial_state)
        self.all_maze_nodes.append(initial_state)

        while not self.not_visited.is_empty():
            current_stack_node: Node = Node(self.not_visited.pop())
            self.time += 1
            current_maze_node: Maze_Node = Maze_Node(current_stack_node.maze_node[0], current_stack_node.maze_node[1])
            self.visited.append(current_maze_node.node)
            self.maze.mark_path(current_maze_node)

            # if the current node is the goal node
            if goal_state == current_maze_node.node:
                # print("goal")
                break
            else:

                # moving vertical -> left
                left_node = Maze_Node(current_maze_node.y, current_maze_node.x - 1).node
                if current_maze_node.x - 1 >= 0 and self.maze.maze[current_maze_node.y][current_maze_node.x - 1] != Cell.BARRIER and left_node not in self.all_maze_nodes:
                    self.not_visited.push(left_node)
                    self.all_maze_nodes.append(left_node)

                # moving vertical -> up
                up_node = Maze_Node(current_maze_node.y - 1, current_maze_node.x).node
                if current_maze_node.y - 1 >= 0 and self.maze.maze[current_maze_node.y - 1][current_maze_node.x] != Cell.BARRIER and up_node not in self.all_maze_nodes:
                    self.not_visited.push(up_node)
                    self.all_maze_nodes.append(up_node)

                # moving vertical -> down
                down_node = Maze_Node(current_maze_node.y + 1, current_maze_node.x).node
                if current_maze_node.y + 1 < self.maze.rows and self.maze.maze[current_maze_node.y + 1][current_maze_node.x] != Cell.BARRIER and down_node not in self.all_maze_nodes:
                    self.not_visited.push(down_node)
                    self.all_maze_nodes.append(down_node)

                # moving vertical -> right
                right_node = Maze_Node(current_maze_node.y, current_maze_node.x + 1).node
                if current_maze_node.x + 1 < self.maze.columns and self.maze.maze[current_maze_node.y][current_maze_node.x + 1] != Cell.BARRIER and right_node not in self.all_maze_nodes:
                    self.not_visited.push(right_node)
                    self.all_maze_nodes.append(right_node)

        print("Final Path: ")
        print(self.maze.print_maze())
        print("Visited nodes list: ", self.visited)
        print("Time to find the goal: ", self.time, "minutes")
        # print("Non visited stack: ", self.not_visited.display())
        # print("all nodes: ", self.all_maze_nodes)


# A* implementation
class PriorityQueue:
    def __init__(self):
        self.priority_queue: List = []

    def is_empty(self) -> bool:
        return not self.priority_queue

    def enqueue(self, node):
        heapq.heappush(self.priority_queue, node)

    def dequeue(self):
        if self.is_empty():
            return "Queue is empty"
        return heapq.heappop(self.priority_queue)

    def display(self):
        if self.is_empty():
            return "Queue is empty"
        print(self.priority_queue)


class A_Star:
    def __init__(self, m1: Maze):
        self.m1: Maze = m1
        self.goal_node: Maze_Node = m1.goal
        # Not visited nodes
        self.non_visited_node_list: PriorityQueue = PriorityQueue()
        # Visited nodes
        self.visited_node_list = []
        self.time: int = 0

    def heuristic_value(self, goal_node: Maze_Node, current_node: Maze_Node):
        self.goal_node: Maze_Node = goal_node
        current_node: Maze_Node = current_node
        heuristic_value: float = max(abs(current_node.heu_node[1][0] - goal_node.heu_node[1][0]), abs(current_node.heu_node[1][1] - goal_node.heu_node[1][1]))
        current_hue_node: Maze_Node = Maze_Node(current_node.y, current_node.x, heuristic_value)
        return current_hue_node.heu_node

    def a_star_algorithm(self):
        initial_state = self.m1.start.heu_node
        goal_state = self.m1.goal.heu_node

        start_maze_node: Maze_Node = Maze_Node(initial_state[1][0], initial_state[1][1])
        goal_maze_node: Maze_Node = Maze_Node(goal_state[1][0], goal_state[1][1])

        start_queue_node = self.heuristic_value(goal_maze_node, start_maze_node)

        self.non_visited_node_list.enqueue(start_queue_node)

        while not self.non_visited_node_list.is_empty():
            current_node = self.non_visited_node_list.dequeue()
            current_maze_node: Maze_Node = Maze_Node(current_node[1][0], current_node[1][1])

            self.time += 1

            self.visited_node_list.append(current_maze_node.heu_node[1])
            self.m1.mark_path(current_maze_node)

            # if the current node is the goal node
            if goal_state == current_maze_node.heu_node:
                print("Final Path: ")
                print(self.m1.print_maze())
                break
            else:

                # moving vertical -> left
                left_node = Maze_Node(current_maze_node.y, current_maze_node.x - 1).heu_node
                left_maze_node: Maze_Node = Maze_Node(left_node[1][0], left_node[1][1])
                if current_maze_node.x - 1 >= 0 and self.m1.maze[current_maze_node.y][current_maze_node.x - 1] != Cell.BARRIER and left_node not in self.visited_node_list:
                    left_queue_node = self.heuristic_value(goal_maze_node, left_maze_node)
                    self.non_visited_node_list.enqueue(left_queue_node)

                # moving vertical -> up
                up_node = Maze_Node(current_maze_node.y - 1, current_maze_node.x).heu_node
                up_maze_node: Maze_Node = Maze_Node(up_node[1][0], up_node[1][1])
                if current_maze_node.y - 1 >= 0 and self.m1.maze[current_maze_node.y - 1][current_maze_node.x] != Cell.BARRIER and up_node not in self.visited_node_list:
                    up_queue_node = self.heuristic_value(goal_maze_node, up_maze_node)
                    self.non_visited_node_list.enqueue(up_queue_node)

                # moving vertical -> down
                down_node = Maze_Node(current_maze_node.y + 1, current_maze_node.x).heu_node
                down_maze_node: Maze_Node = Maze_Node(down_node[1][0], down_node[1][1])
                if current_maze_node.y + 1 < self.m1.rows and self.m1.maze[current_maze_node.y + 1][current_maze_node.x] != Cell.BARRIER and down_node not in self.visited_node_list:
                    down_queue_node = self.heuristic_value(goal_maze_node, down_maze_node)
                    self.non_visited_node_list.enqueue(down_queue_node)

                # moving vertical -> right
                right_node = Maze_Node(current_maze_node.y, current_maze_node.x + 1).heu_node
                right_maze_node: Maze_Node = Maze_Node(right_node[1][0], right_node[1][1])
                if current_maze_node.x + 1 < self.m1.columns and self.m1.maze[current_maze_node.y][current_maze_node.x + 1] != Cell.BARRIER and right_node not in self.visited_node_list:
                    right_queue_node = self.heuristic_value(goal_maze_node, right_maze_node)
                    self.non_visited_node_list.enqueue(right_queue_node)

        print("Visited nodes list: ", self.visited_node_list)
        print("Time to find the goal: ", self.time, "minutes")


if __name__ == "__main__":

    start_node = Maze_Node(random.randint(0, 5), random.randint(0, 5))
    end = Maze_Node(random.randint(0, 5), random.randint(0, 5))

    barriers_list = []
    for i in range(6):
        n: Maze_Node = Maze_Node(random.randint(0, 5), random.randint(0, 5))
        barriers_list.append(n)
    m = Maze(6, 6, barriers_list, start_node, end)

    print("----------Starting Maze----------")
    print(m.print_maze())

    print("----------A* Searching Algorithm----------")
    a_star = A_Star(m)
    a_star.a_star_algorithm()

    print()
    print("----------DFS Searching Algorithm----------")
    dfs = DFS(m)
    dfs.dfs_algorithm()
