import heapq
import copy

# Define the goal state for the 8-puzzle
GOAL_STATE = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]

class Node:
    """Represents a node in the search tree"""
    def __init__(self, state, parent=None, action=None, depth=0, heuristic=0):
        self.state = state        # Current puzzle configuration (3x3 grid)
        self.parent = parent      # Pointer to parent node
        self.action = action      # Move that generated this node ('up', 'down', etc.)
        self.depth = depth        # Depth/Cost from initial state (g(n))
        self.heuristic = heuristic  # Heuristic value (h(n))
        self.cost = depth + heuristic  # Total estimated cost (f(n) = g(n) + h(n))

    def __lt__(self, other):
        """Override less-than for priority queue ordering"""
        return self.cost < other.cost

    def __eq__(self, other):
        """Check if two nodes have identical states"""
        return self.state == other.state

def find_blank(state):
    """Find coordinates of the blank tile (0) in the puzzle"""
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                return (i, j)
    return None  # Should never happen for valid puzzles

def move(state, direction):
    """Generate new state by moving blank tile in given direction"""
    i, j = find_blank(state)
    new_state = copy.deepcopy(state)  # Create copy to avoid modifying original
    
    # Swap blank tile with adjacent tile based on direction
    if direction == 'up' and i > 0:
        new_state[i][j], new_state[i-1][j] = new_state[i-1][j], new_state[i][j]
        return new_state
    elif direction == 'down' and i < 2:
        new_state[i][j], new_state[i+1][j] = new_state[i+1][j], new_state[i][j]
        return new_state
    elif direction == 'left' and j > 0:
        new_state[i][j], new_state[i][j-1] = new_state[i][j-1], new_state[i][j]
        return new_state
    elif direction == 'right' and j < 2:
        new_state[i][j], new_state[i][j+1] = new_state[i][j+1], new_state[i][j]
        return new_state
    else:
        return None  # Invalid move

def get_children(node):
    """Generate all valid child nodes from current state"""
    children = []
    directions = ['up', 'down', 'left', 'right']
    
    for direction in directions:
        child_state = move(node.state, direction)
        if child_state is not None:
            # Create new node with updated depth and inherited state
            children.append(Node(child_state, node, direction, node.depth + 1, 0))
    return children

def misplaced_tile_heuristic(state):
    """Calculate number of tiles not in their goal position (excluding blank)"""
    count = 0
    for i in range(3):
        for j in range(3):
            # Compare current position with goal position
            if state[i][j] != GOAL_STATE[i][j] and state[i][j] != 0:
                count += 1
    return count

def manhattan_distance_heuristic(state):
    """Calculate sum of Manhattan distances for all tiles to their goal positions"""
    distance = 0
    for i in range(3):
        for j in range(3):
            value = state[i][j]
            if value != 0:
                # Calculate correct position using division and modulus
                goal_row, goal_col = divmod(value - 1, 3)
                distance += abs(i - goal_row) + abs(j - goal_col)
    return distance

def general_search(initial_state, heuristic_func):
    """Core search algorithm implementing UCS or A*"""
    visited = set()  # Track visited states to prevent cycles
    initial_node = Node(initial_state)
    
    # Initialize heuristic if using A*
    if heuristic_func:
        initial_node.heuristic = heuristic_func(initial_state)
    initial_node.cost = initial_node.depth + initial_node.heuristic
    
    # Priority queue ordered by node cost
    heap = []
    heapq.heappush(heap, initial_node)
    
    nodes_expanded = 0
    max_queue_size = 1

    while heap:
        max_queue_size = max(max_queue_size, len(heap))
        current_node = heapq.heappop(heap)
        
        # Print the current state being expanded
        print(f"The best state to expand with a g(n) = {current_node.depth} and h(n) = {current_node.heuristic} is...")
        for row in current_node.state:
            print(row)
        print()
        
        # Check if current state matches goal
        if current_node.state == GOAL_STATE:
            return current_node, nodes_expanded, max_queue_size
        
        # Convert state to tuple for hashable set storage
        state_tuple = tuple(map(tuple, current_node.state))
        if state_tuple in visited:
            continue
        visited.add(state_tuple)
        
        nodes_expanded += 1
        children = get_children(current_node)
        
        for child in children:
            # Calculate heuristic if using A*
            if heuristic_func:
                child.heuristic = heuristic_func(child.state)
            child.cost = child.depth + child.heuristic
            
            # Check if child state hasn't been visited
            child_tuple = tuple(map(tuple, child.state))
            if child_tuple not in visited:
                heapq.heappush(heap, child)
                
    return None, nodes_expanded, max_queue_size

def print_solution(node):
    """Backtrack and print solution path from goal to initial state"""
    path = []
    while node:
        path.append(node)
        node = node.parent  # Move up the tree
    
    # Reverse to show path from initial to goal
    for t in reversed(path):
        print(f"Move {t.action}" if t.action else "Initial State")
        for row in t.state:
            print(row)
        print()

def main():
    """Main user interface for puzzle solver"""
    print("Welcome to the 8-Puzzle Solver!")
    choice = input("Enter '1' for a default puzzle or '2' to input your own: ")
    
    if choice == '1':
        # Default puzzle (Can Modify)
        puzzle = [[1, 2, 3], [4, 0, 6], [7, 5, 8]]
    else:
        print("Enter your puzzle row by row (use 0 for the blank):")
        puzzle = []
        for i in range(3):
            row = list(map(int, input(f"Row {i+1}: ").split()))
            puzzle.append(row)
    
    algorithm = input("Choose algorithm (1: UCS, 2: A* Misplaced, 3: A* Manhattan): ")
    heuristic = None
    
    # Assign heuristic function based on user choice
    if algorithm == '2':
        heuristic = misplaced_tile_heuristic
    elif algorithm == '3':
        heuristic = manhattan_distance_heuristic
    
    # Execute search algorithm
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