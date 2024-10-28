import heapq

# Helper function to calculate the Manhattan Distance heuristic
def manhattanDistance(state, goal):
    distance = 0
    for i in range(len(state)):
        # Ignore the blank space
        if state[i] != 0:  
            goalPos = goal.index(state[i])
            distance += abs(i // 3 - goalPos // 3) + abs(i % 3 - goalPos % 3)
    return distance

# Function to get neighbors (possible moves)
def getNeighbors(state):
    neighbors = []
    # Find the position of the blank tile (0)
    zeroPos = state.index(0)  
    row, col = zeroPos // 3, zeroPos % 3
    
    # Possible moves (up, down, left, right)
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    for dr, dc in moves:
        newRow, newCol = row + dr, col + dc
        if 0 <= newRow < 3 and 0 <= newCol < 3:
            newZeroPos = newRow * 3 + newCol
            newState = list(state)
            newState[zeroPos], newState[newZeroPos] = newState[newZeroPos], newState[zeroPos]
            neighbors.append(newState)
    
    return neighbors

# A* algorithm to solve the 8-puzzle
def aStar(start, goal):
    # Priority queue to store states, sorted by f = g + h
    priorityQueue = []
    # (f, state, g, path)
    heapq.heappush(priorityQueue, (0, start, 0, []))  
    
    # To track visited states
    visited = set()  
    visited.add(tuple(start))
    
    while priorityQueue:
        f, current, g, path = heapq.heappop(priorityQueue)
        
        # If the goal state is reached, return the path
        if current == goal:
            return path + [current]
        
        # Explore neighbors (possible moves)
        for neighbor in getNeighbors(current):
            if tuple(neighbor) not in visited:
                visited.add(tuple(neighbor))
                # Heuristic
                h = manhattanDistance(neighbor, goal)  
                heapq.heappush(priorityQueue, (g + 1 + h, neighbor, g + 1, path + [current]))
    
    # Return None if no solution is found
    return None  

# Function to print the puzzle states
def printSolutionPath(solution):
    steps = 0
    if solution:
        for step in solution:
            print(step[0:3])
            print(step[3:6])
            print(step[6:9])
            print()
            steps = steps + 1
    else:
        print("No solution found.")
    
    print("Total steps:", steps)

# Main function to orchestrate the puzzle solving
if __name__ == "__main__":
    
    # Random initial state
    start = [0, 2, 4, 6, 1, 3, 7, 8, 5]
    # Goal state  
    goal = [1, 2, 3, 4, 5, 6, 7, 8, 0]   

    # Solve the puzzle using A* and Manhattan Distance heuristic
    solution = aStar(start, goal)

    # Print the solution path
    printSolutionPath(solution)
