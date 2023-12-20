import re
import time
import numpy as np

GRID_SIZE = 12
WALL = -1

def check_for_match(match):
    if match:
        return int(match[0])
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
            mission = re.findall(r'(\w\d)', line)
            missions += [mission]
    return missions

def write_output(makespan, completion_times, execution_time, path, file='output.txt'):
    with open(file, 'w') as f:
        f.write("//Quantitative values\n")
        f.write(f'{makespan - 1}\t//Makespan\n')
        
        # Show the completion time for each hauler
        for i, completion_time in enumerate(completion_times):
            f.write(f'{completion_time}\t//Mission completion time hauler {i+1}\n')
            
        # Show the execution time
        f.write(f'{execution_time:.4f}\t//Application execution time (in millisecond)\n')
        f.write("//Path to the final destination\n")
        
        # Create makespan x nHaulers matrix
        for i in range(makespan):
            f.write(f'{i}')

            # Get the pos of each hauler or the last pos
            try:
                x, y = path[i]
                f.write(f',[{x + 1},{y + 1}]')
            except:
                x, y = path[-1]
                f.write(f',[{x + 1},{y + 1}]')
            f.write('\n')

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
                # Check if the new path 
                # \bis shorter
                if distance[current] + 1 < distance[x,y]:
                    distance[x,y] = distance[current] + 1
                    pred[x,y] = current
            
    return distance


def find_path(start_pos, end_pos, path, pred):
    x, y = end_pos
    x_start, y_start = start_pos
    # Walk back from the end point to the start point
    while pred[x,y] != (x_start, y_start):
        path.append([x,y])
        pos = pred[x,y]
        x, y = pos
    # Add last and start point
    path.append([x,y])
    path.reverse()
    return path

def part1(config, mission):
    # Read configuration files
    nHaulers, nLP, nULP, nSO, nCS, hauler_positions, LP_positions, ULP_positions, SO_positions, CS_positions, max_energy, initial_energy = config
    mission = mission[0]
    
    # Convert mission to coordinates
    mission_pos = [()] * len(mission)
    for i, pos in enumerate(mission):
        match pos[0]:
            case 'L':
                mission_pos[i] = LP_positions[int(pos[1])-1]
            case 'U':
                mission_pos[i] = ULP_positions[int(pos[1])-1]
            case _:
                print("Error: Mission not found")
                    

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
    
    final_path = []
    mission_path = []
    mission_paths = [] 
    completion_time = 0
    
    # ----------------------------------------
    # Start path finding
    start = time.perf_counter()

    paths = []
    start_pos = hauler_positions[0]
    # Get the mission positions for the hauler
    for next_pos in mission_pos:
        # Use A* to find the path
        last_dist = astar(start_pos, next_pos, grid_map, distance, pred)
        # Find the path
        path =[]
        path = find_path(start_pos, next_pos, path, pred)
        if start_pos == hauler_positions[0]:
            path.insert(0, start_pos)
        # Add path to paths and update start_pos
        start_pos = next_pos
        paths.append(path)
        mission_path = mission_path  + path
        # Reset distance
        distance = clean_distance.copy()
    
    # Complete the mission
    completion_time = len(mission_path)
    
    mission_paths  = paths.copy()
    final_path = mission_path
    
    # Caclulate the total distance
    completion_time = [len(final_path) - 1]
    makespan = max(completion_time)

    # Stop path finding
    end = time.perf_counter()
    execution_time = (end - start)*1000
    print(f"{execution_time = :.2f} ms")
        
    write_output(makespan + 1, completion_time, execution_time, final_path)
    
if __name__ == "__main__":
    config = read_config()
    mission = read_mission()
    part1(config, mission)