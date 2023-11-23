import re
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors
from matplotlib import animation

GRID_SIZE = 18
WALL = -1

def check_for_match(match):
    if match:
        return match.group(1)
    else:
        return 0
    
def read_config(file='config.txt'):
    with open(file, 'r') as f:
        lines = f.readlines()
        # Get the number of items
        nHaulers = re.search(r'(\d+)', lines[0]).group(1)
        nLP = re.search(r'(\d+)', lines[1]).group(1)
        nULP = re.search(r'(\d+)', lines[2]).group(1)
        nSO = re.search(r'(\d+)', lines[3]).group(1)
        nCS = re.search(r'(\d+)', lines[4]).group(1)
        
        # Get their positions
        init_haulers = re.findall(r'\[(\d+),(\d+)\]', lines[5])
        LP_positions = re.findall(r'\[(\d+),(\d+)\]', lines[6])
        ULP_positions = re.findall(r'\[(\d+),(\d+)\]', lines[7])
        SO_positions = re.findall(r'\[(\d+),(\d+)\]', lines[8])
        CS_positions = re.findall(r'\[(\d+),(\d+)\]', lines[9])
        
        # Transform the positions to integers
        init_haulers = [[int(x), int(y)] for x, y in init_haulers]
        LP_positions = [[int(x), int(y)] for x, y in LP_positions]
        ULP_positions = [[int(x), int(y)] for x, y in ULP_positions]
        SO_positions = [[int(x), int(y)] for x, y in SO_positions]
        CS_positions = [[int(x), int(y)] for x, y in CS_positions]
        
        # Remove offset from pos
        init_haulers = [[x-1, y-1] for x, y in init_haulers]
        LP_positions = [[x-1, y-1] for x, y in LP_positions]
        ULP_positions = [[x-1, y-1] for x, y in ULP_positions]
        SO_positions = [[x-1, y-1] for x, y in SO_positions]
        CS_positions = [[x-1, y-1] for x, y in CS_positions]
        
        # Get the max battery capacity and initial energy
        max_energy = check_for_match(re.findall(r'(\d+)', lines[10]))
        initial_energy = check_for_match(re.findall(r'(\d+)', lines[11]))
        
    return nHaulers, nLP, nULP, nSO, nCS, init_haulers, LP_positions, ULP_positions, SO_positions, CS_positions, max_energy, initial_energy
    
def read_mission(file='mission.txt'):
    with open(file, 'r') as f:
        lines = f.readlines()
        # Read the mission
        missions = []
        for line in lines:
            print(line)
            mission = re.findall(r'(\w\d)', line)
            missions += [mission]
    return missions


def bfs(start, end, grid_map, distance, pred, distance_plt):
    # Unpack start and end
    start_x, start_y = start
    end_x, end_y = end
    
    # BFS
    visited = np.full((GRID_SIZE, GRID_SIZE), False)
    queue = []
    bfs = []
    
    # Start from Hauler position
    queue.append((start_x, start_y))
    visited[start_x, (start_y)] = True
    
    while queue:
        s = queue.pop(0)
        bfs.append(s)
        
        # On reaching the end point, stop
        if s == (end_x, end_y):
            distance[end_x, end_y] = -2
            break
        
        x, y = s
        if x > 0 and grid_map[x-1, y] != WALL and not visited[x-1,y]:
            queue.append((x-1, y))
            visited[x-1,y] = True
            distance[x-1,y] = distance[x,y] + 1
            pred[x-1,y] = s
        if x < GRID_SIZE-1 and grid_map[x+1, y] != WALL and not visited[x+1,y]:
            queue.append((x+1, y))
            visited[x+1,y] = True
            distance[x+1,y] = distance[x,y] + 1
            pred[x+1,y] = s
        if y > 0 and grid_map[x, y-1] != WALL and not visited[x,y-1]:
            queue.append((x, y-1))
            visited[x,y-1] = True
            distance[x,y-1] = distance[x,y] + 1
            pred[x,y-1] = s
        if y < GRID_SIZE-1 and grid_map[x, y+1] != WALL and not visited[x,y+1]:
            queue.append((x, y+1))
            visited[x,y+1] = True
            distance[x,y+1] = distance[x,y] + 1
            pred[x,y+1] = s
        
        # After each find plot the distance
        distance_plt.imshow(distance, cmap='hot', interpolation='nearest')
        plt.pause(0.025)
        
    return distance

def astar(start, end, grid_map, distance, pred):
    # Unpack start and end
    start_x, start_y = start
    end_x, end_y = end
    
    # A Star
    openQueue = []
    closedQueue = []
    
    openQueue.append((start_x, start_y))
    while openQueue:
        # Find the node with the lowest cost
        min_cost = 1000
        for node in openQueue:
            x, y = node
            # Get G cost
            cost = distance[x,y]
            # Get H cost (Manhattan distance)
            cost += abs(end_x - x) + abs(end_y - y)
            # Check if cost is lower than min_cost
            if cost < min_cost:
                min_cost = cost
                current = node
        # Remove the node from the open queue
        openQueue.remove(current)
        # Add the node to the closed queue
        closedQueue.append(current)
        
        # Check if the end is reached
        if current == (end_x, end_y):
            return distance
        
        # Check the neighbors
        x, y = current
        for neighbor in [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]:
            # Check if neighbor is in closed queue, otherwise skip
            if neighbor in closedQueue:
                continue
            
            # Check ranges
            x, y = neighbor
            if x < 0 or x > GRID_SIZE-1 or y < 0 or y > GRID_SIZE-1:
                continue
            
            # Check if neighbor is a wall
            if grid_map[x,y] == WALL:
                continue
            
            if not neighbor in openQueue:
                openQueue.append(neighbor)
                distance[x,y] = distance[current] + 1
                pred[x,y] = current
            else:
                # Check if the new path is shorter
                if distance[current] + 1 < distance[x,y]:
                    distance[x,y] = distance[current] + 1
                    pred[x,y] = current
                    
            
    return distance

def find_path(start_pos, end_pos, path, pred):
    x, y = end_pos
    x_start, y_start = start_pos
    # Walk back from the end point to the start point
    while pred[x,y] != (x_start, y_start):
        path.append((x,y))
        pos = pred[x,y]
        x, y = pos
    # Add last and start point
    path.append((x,y))
    path.append((x_start, y_start))
    
    return path

if __name__ == "__main__":
    nHaulers, nLP, nULP, nSO, nCS, hauler_positions, LP_positions, ULP_positions, SO_positions, CS_positions, max_energy, initial_energy = read_config()
    
    missions = read_mission()
    missions_pos = [[] for i in range(len(missions))]
    
    # Convert mission to coordinates
    for hauler_id, mission in enumerate(missions):
        mission_pos = [[] for i in range(len(mission))]
        for i, pos in enumerate(mission):
            match pos[0]:
                case 'L':
                    mission_pos[i] = LP_positions[int(pos[1])-1]
                case 'U':
                    mission_pos[i] = ULP_positions[int(pos[1])-1]
                case 'S':
                    mission_pos[i] = SO_positions[int(pos[1])-1]
                case 'C':
                    mission_pos[i] = CS_positions[int(pos[1])-1]
                case _:
                    print("Error: Mission not found")
                    
        missions_pos[hauler_id] = mission_pos
    
    print(f"{missions=} {missions_pos=}")
    
    # Create map and distance, pred for path finding
    grid_map = np.zeros((GRID_SIZE, GRID_SIZE))
    distance = np.zeros((GRID_SIZE, GRID_SIZE))
    pred = {}
    
    # Add Walls, only on distances for graph
    for so in SO_positions:
        grid_map[so[0], so[1]] = WALL
        distance[so[0], so[1]] = WALL
    
    # Reset distance after each found point
    clean_distance = distance.copy()
    
    # Setup figure and image for grid
    fig_grid, ax_grid = plt.subplots(figsize=(10,10))
    ax_grid.imshow(grid_map, cmap='hot', interpolation='nearest')

    # Setup figure and image for distance
    fig_distance, ax_distance = plt.subplots(figsize=(10,10))
    ax_distance.imshow(distance, cmap='hot', interpolation='nearest')
    
    # Start path finding
    paths = []
    for hauler_id, mission in enumerate(missions_pos):
        start_pos = hauler_positions[hauler_id]
        # Loop over the mission positions
        for next_pos in mission:
            # Use BFS to map the distance
            last_dist = astar(start_pos, next_pos, grid_map, distance, pred)
            # Find the path
            path =[]
            path = find_path(start_pos, next_pos, path, pred)
            # Add path to paths and update start_pos
            start_pos = next_pos
            paths.append(path)
            # Reset distance
            distance = clean_distance.copy()
            
            # Plot the path
            for nPath, path in enumerate(paths):
                for pos in path:
                    x, y = pos
                    grid_map[x,y] = nPath+1


            
    for pos in path:
        x, y = pos
        last_dist[x,y] = -2
    
    # Show the grid
    ax_grid.imshow(grid_map, cmap='terrain', interpolation='nearest', vmin=-1, vmax=6)
    ax_distance.imshow(last_dist, cmap='terrain', interpolation='nearest', vmin=-2, vmax=GRID_SIZE*2)
    plt.show()