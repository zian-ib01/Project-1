import heapq
import copy

# The goal state of the 8-puzzle (how it should look when solved)
GOAL_STATE = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]

class Node:
    """Represents a puzzle state in the search tree."""
    def __init__(self, state, parent=None, action=None, depth=0, heuristic=0):
        self.state = state        # Current layout of the puzzle
        self.parent = parent      # Parent node (previous state)
        self.action = action      # Move that led to this state ('up', 'down', etc.)
        self.depth = depth        # Steps taken from the start (g(n))
        self.heuristic = heuristic  # Estimated cost to reach the goal (h(n))
        self.cost = depth + heuristic  # Total estimated cost (f(n) = g(n) + h(n))

    def __lt__(self, other):
        """Allows the priority queue to compare nodes based on cost."""
        return self.cost < other.cost

    def __eq__(self, other):
        """Checks if two nodes have the same puzzle layout."""
        return self.state == other.state

def find_blank(state):
    """Finds the row and column of the blank tile (0)."""
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                return (i, j)
    return None  # This shouldn't happen in a valid puzzle

def move(state, direction):
    """Moves the blank tile in the specified direction if possible."""
    i, j = find_blank(state)
    new_state = copy.deepcopy(state)  # Copy the state to avoid modifying the original
    
    if direction == 'up' and i > 0:
        new_state[i][j], new_state[i-1][j] = new_state[i-1][j], new_state[i][j]
    elif direction == 'down' and i < 2:
        new_state[i][j], new_state[i+1][j] = new_state[i+1][j], new_state[i][j]
    elif direction == 'left' and j > 0:
        new_state[i][j], new_state[i][j-1] = new_state[i][j-1], new_state[i][j]
    elif direction == 'right' and j < 2:
        new_state[i][j], new_state[i][j+1] = new_state[i][j+1], new_state[i][j]
    else:
        return None  # Move not allowed
    
    return new_state

def get_children(node):
    """Generates all possible valid moves from the current state."""
    children = []
    directions = ['up', 'down', 'left', 'right']
    
    for direction in directions:
        child_state = move(node.state, direction)
        if child_state is not None:
            children.append(Node(child_state, node, direction, node.depth + 1, 0))
    return children

def misplaced_tile_heuristic(state):
    """Counts how many tiles are in the wrong position."""
    count = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] != GOAL_STATE[i][j] and state[i][j] != 0:
                count += 1
    return count

def manhattan_distance_heuristic(state):
    """Calculates the total distance each tile is from its goal position."""
    distance = 0
    for i in range(3):
        for j in range(3):
            value = state[i][j]
            if value != 0:
                goal_row, goal_col = divmod(value - 1, 3)
                distance += abs(i - goal_row) + abs(j - goal_col)
    return distance

def general_search(initial_state, heuristic_func):
    """Runs the search algorithm (UCS or A* depending on the heuristic)."""
    visited = set()  # Keeps track of explored states
    initial_node = Node(initial_state)
    
    if heuristic_func:
        initial_node.heuristic = heuristic_func(initial_state)
    initial_node.cost = initial_node.depth + initial_node.heuristic
    
    heap = []  # Priority queue to store nodes by cost
    heapq.heappush(heap, initial_node)
    
    nodes_expanded = 0
    max_queue_size = 1

    while heap:
        max_queue_size = max(max_queue_size, len(heap))
        current_node = heapq.heappop(heap)
        
        print(f"Expanding state with g(n) = {current_node.depth} and h(n) = {current_node.heuristic}:")
        for row in current_node.state:
            print(row)
        print()
        
        if current_node.state == GOAL_STATE:
            return current_node, nodes_expanded, max_queue_size
        
        state_tuple = tuple(map(tuple, current_node.state))
        if state_tuple in visited:
            continue
        visited.add(state_tuple)
        
        nodes_expanded += 1
        children = get_children(current_node)
        
        for child in children:
            if heuristic_func:
                child.heuristic = heuristic_func(child.state)
            child.cost = child.depth + child.heuristic
            
            child_tuple = tuple(map(tuple, child.state))
            if child_tuple not in visited:
                heapq.heappush(heap, child)
                
    return None, nodes_expanded, max_queue_size

def print_solution(node):
    """Prints the sequence of moves from start to goal state."""
    path = []
    while node:
        path.append(node)
        node = node.parent
    
    for t in reversed(path):
        print(f"Move: {t.action}" if t.action else "Initial State")
        for row in t.state:
            print(row)
        print()

def main():
    """Main user interface for puzzle solver"""
    print("Welcome to the 8-Puzzle Solver!")
    choice = input("Enter '1' for a default puzzle or '2' to enter your own: ")
    
    if choice == '1':
        puzzle = [[1, 2, 3], [4, 0, 6], [7, 5, 8]]  # Example starting state
    else:
        print("Enter your puzzle row by row (use 0 for the blank space):")
        puzzle = [list(map(int, input(f"Row {i+1}: ").split())) for i in range(3)]
    
    algorithm = input("Choose an algorithm (1: UCS, 2: A* Misplaced, 3: A* Manhattan): ")
    heuristic = None
    
    if algorithm == '2':
        heuristic = misplaced_tile_heuristic
    elif algorithm == '3':
        heuristic = manhattan_distance_heuristic
    
    solution, nodes_expanded, max_queue = general_search(puzzle, heuristic)
    
    if solution:
        print("Goal state reached!")
        print_solution(solution)
        print(f"Solution depth: {solution.depth}")
    else:
        print("No solution found.")
    
    print(f"Nodes expanded: {nodes_expanded}")
    print(f"Max queue size: {max_queue}")

if __name__ == "__main__":
    main()