import re
import time
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors
from matplotlib import animation

GRID_SIZE = 12
WALL = -1
ENERGY_COST = 10 * 5
CHARGE_TIME = 5

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

def write_output(makespan, completion_times, execution_time, paths, file='output.txt'):
    with open(file, 'w') as f:
        f.write("//Quantitative values\n")
        f.write(f'{makespan - 1}\t//Makespan\n')
        
        # Show the completion time for each hauler
        for i, completion_time in enumerate(completion_times):
            f.write(f'{completion_time}\t//Mission completion time hauler {i+1}\n')
            
        # Show the execution time
        f.write(f'{execution_time:.4f}\t//Application execution time (in millisecond)\n')
        f.write("//Path to the final destination\n")
        
        # Increase by one as the grid starts at 1
        for path in paths:
            for pos in path:
                pos[0] += 1
                pos[1] += 1
        # Create makespan x nHaulers matrix
        for i in range(makespan):
            f.write(f'{i}')

            # Get the pos of each hauler or the last pos
            for path in paths:
                try:
                    f.write(f',{path[i]}')
                except:
                    f.write(f',{path[-1]}')
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
            # Get H cost (Euclidean distance)
            cost += np.sqrt((end_x - x)**2 + (end_y - y)**2)
            
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

def distance_to_charger(charger_pos, mission_pos, grid_map, distance, pred):
    # Start at charger point
    start_x, start_y = charger_pos
    reached = 0
    visited = np.full((GRID_SIZE, GRID_SIZE), fill_value=False, dtype=bool)
    # Use BFS to find the shortest path to the mission points
    queue = []
    queue.append((start_x, start_y))
    while queue:
        s = queue.pop(0)
        x, y = s
        visited[x,y] = True
            
        # Check neighbors
        for neighbor in [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]:
            # Check if not outside ranges
            x, y = neighbor
            if x < 0 or x > GRID_SIZE-1 or y < 0 or y > GRID_SIZE-1:
                continue
            
            # Check if neighbor is a wall
            if grid_map[x,y] == WALL:
                continue
            
            # If not visited add it to the queue
            if not visited[x,y] and not neighbor in queue:
                queue.append(neighbor)
                pred[x, y] = s
                distance[x, y] = distance[s] + 1
                
                # Check if the neighbor is end point
                for mission in mission_pos.values():
                    if neighbor == tuple(mission):
                        reached += 1
                    if reached == len(mission_pos):
                        return distance

    return distance

def find_shortest_path_energy(max_cap, init_cap, total_cost, next_cost, end_cost, charger_cost, charger_next_cost, mission):
    queue = []
    path = []
    curr_cap = [0] * len(mission)
    curr_cap[0] = init_cap
    
    # No need to add charging, can do missions with initial capacity
    if init_cap >= total_cost:
        return mission
    
    # Find the nodes which can be reached with the current capacity
    for i in range(len(mission)):
        if curr_cap[i] > (next_cost[i] + charger_cost[i]):
            curr_cap[i+1] = curr_cap[i] - next_cost[i]
            queue.append(i)

    while queue:
        idx = queue[0]
        # Find the node with the lowest cost
        min_cost = 1000000
        for i in queue:
            # If the end can be reached, goto end
            if curr_cap[i] > end_cost[i]:
                path += [j for j in range(i, len(mission))]
                # path = [mission[j] if isinstance(j, int) else j for j in path]
                return path
            
            # Calculate the cost
            cost = curr_cap[i] - charger_cost[i] + max_cap + charger_next_cost[i]
            if cost < min_cost:
                min_cost = cost
                current = i

        # Move to charger
        for i in range(idx, current):
            path.append(i)
        path += [current, 'CS']
        curr_cap[current + 1] = max_cap - charger_next_cost[current]
        # Check if the next node is the last, otherwise add it to the queue
        if current + 1 <= len(mission):
            queue = [current + 1]
        else:
            # path = [mission[j] if isinstance(j, int) else j for j in path]
            return path


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
    # path.append([x_start,y_start])
    path.reverse()
    return path

if __name__ == "__main__":
    # Read configuration files
    nHaulers, nLP, nULP, nSO, nCS, hauler_positions, LP_positions, ULP_positions, SO_positions, CS_positions, max_energy, initial_energy = read_config(file='battery_config.txt')
    missions = read_mission(file='battery_mission.txt')
    missions_pos = [[] for i in range(len(missions))]
    
    unique_missions = {}
    # Convert mission to coordinates
    for hauler_id, mission in enumerate(missions):
        mission_pos = [()] * len(mission)
        for i, pos in enumerate(mission):
            match pos[0]:
                case 'L':
                    mission_pos[i] = LP_positions[int(pos[1])-1]
                case 'U':
                    mission_pos[i] = ULP_positions[int(pos[1])-1]
                case _:
                    print("Error: Mission not found")
                    
        missions_pos[hauler_id] = mission_pos
    for mission, mission_pos in zip(missions, missions_pos):
        for id, pos in zip(mission, mission_pos):
            unique_missions[id] = pos

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
    
    final_path = [[]] * len(missions)
    final_paths = [[]] * len(missions)
    completion_times = [0] * len(missions)
    
    # ----------------------------------------
    # Start path finding
    start = time.perf_counter()
    # Get the mission for each hauler
    for hauler_id, mission in enumerate(missions_pos):
        paths = []
        start_pos = hauler_positions[hauler_id]
        # Get the mission positions for the hauler
        for next_pos in mission:
            # Use A* to find the path
            last_dist = astar(start_pos, next_pos, grid_map, distance, pred)
            # Find the path
            path =[]
            path = find_path(start_pos, next_pos, path, pred)
            # Add path to paths and update start_pos
            start_pos = next_pos
            paths.append(path)
            final_path[hauler_id] = final_path[hauler_id]  + path
            # Reset distance
            distance = clean_distance.copy()
        
        # Complete the mission
        completion_times[hauler_id] = len(final_path[hauler_id])
        
        final_paths[hauler_id]  = paths.copy()
        final_path[hauler_id] = [hauler_positions[hauler_id]] + final_path[hauler_id]

    if CS_positions:
        CS_position = CS_positions[0]
        mission = missions[0]
        # ----------------------------------------
        # Find the distance to the charger
        pred = {}
        distance = distance_to_charger(CS_position, unique_missions, grid_map, distance, pred)
        # Find the path from charger to mission points
        charger_paths = {}
        for id, pos in unique_missions.items():
            path =[]
            path = find_path(CS_position, pos, path, pred)
            charger_paths[id] = [CS_position] + path
        
        # print(f"{charger_paths = }")
        # ----------------------------------------
        # Create graph for hauler with charger
        graph = []
        total_energy_cost = len(final_path[0]) * ENERGY_COST
        prev_energy_cost = total_energy_cost
        next_cost = []
        end_cost = []
        charger_cost = []
        charger_next_cost = []
        
        for i,id in enumerate(mission):
            try:
                next_cost.append(len(paths[i]) * ENERGY_COST)
                charger_cost.append(len(charger_paths[id]) * ENERGY_COST)
                charger_next_cost.append(len(charger_paths[mission[i+1]]) * ENERGY_COST)
                end_cost.append(prev_energy_cost)
                prev_energy_cost -= next_cost[-1]
            except:
                continue
        
        # ----------------------------------------
        # Find the shortest path with energy
        charger_mission = find_shortest_path_energy(max_energy, initial_energy, total_energy_cost, next_cost, end_cost, charger_cost, charger_next_cost, mission)
        # Create the final path
        if not charger_mission == mission:
            for i,val in enumerate(charger_mission):
                if val is 'CS':
                    charger_paths[mission[i-1]].reverse()
                    final_path[0] += (charger_paths[mission[i-1]])
                    final_path[0] += (charger_paths[mission[i]])
                else:
                    final_path[0] += final_paths[0][i]
                    if i is len(mission) - 1:
                        break
        final_path[0] = [hauler_positions[0]] + final_path[0]
    # Stop path finding
    end = time.perf_counter()
    execution_time = (end - start)*1000
    print(f"{execution_time = :.2f} ms")
    # # Setup figure and image for grid
    # fig_grid, ax_grid = plt.subplots(figsize=(10,10))
    # ax_grid.imshow(grid_map, cmap='hot', interpolation='nearest')

    # Setup figure and image for distance
    # fig_distance, ax_distance = plt.subplots(figsize=(10,10)) 
    # ax_distance.imshow(distance, cmap='hot', interpolation='nearest')
    
    # # Show the grid
    # ax_grid.imshow(grid_map, cmap='terrain', interpolation='nearest', vmin=-1, vmax=6)
    # ax_distance.imshow(last_dist, cmap='terrain', interpolation='nearest', vmin=-2, vmax=GRID_SIZE*2)
    plt.show()
    
    # Caclulate the total distance
    path_len = [len(path) for path in final_path]
    makespan = max(path_len)
    
    # print(f"{makespan = }")
    # for i, path in enumerate(final_path):
    #     print(f"{i+1} ({len(path)}): {path}")
        
    write_output(makespan, completion_times, execution_time, final_path)