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
        
        # Create makespan x nHaulers matrix
        for i in range(makespan):
            f.write(f'{i}')

            # Get the pos of each hauler or the last pos
            for path in paths:
                try:
                    x, y = path[i]
                    f.write(f',[{x + 1},{y + 1}]')
                except:
                    x, y = path[-1]
                    f.write(f',[{x + 1},{y + 1}]')
                    # f.write(f'[0,0]')
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
    
    queue = [0]
    while queue:
        start = queue[0]
        # Find the node with the lowest cost
        min_cost = 1000000
        for i in queue:
            # If the end can be reached, goto end
            if curr_cap[i] >= end_cost[i] and i <= len(mission):
                path += [j for j in range(i, len(mission))]
                return path
            
            # Calculate the cost
            recharger_cost = max_cap * int((end_cost[i] - curr_cap[i])/ max_cap)
            cost = charger_cost[i] + recharger_cost - (max_cap - (charger_next_cost[i] - curr_cap[i]))
            
            can_reach_charger = int(curr_cap[i]/ENERGY_COST) > int(charger_cost[i]/ENERGY_COST + 1)
            if not can_reach_charger:
                cost = 1000000
            
            if cost < min_cost:
                min_cost = cost
                current = i
            
            can_reach_next = int(curr_cap[i]/ENERGY_COST) > int(next_cost[i]/ENERGY_COST + 1)
            if can_reach_next and not i+1 in queue and i+2 < len(mission):
                queue.append(i+1)
                curr_cap[i+1] = curr_cap[i] - next_cost[i]
                
        # Move to charger
        for i in range(start, current):
            path.append(i)
        path += [current, 'CS']
        curr_cap[current + 1] = max_cap - charger_next_cost[current]
        # Check if the next node is the last, otherwise add it to the queue
        if current + 1 <= len(mission):
            queue = [current + 1]
        else:
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

def part2(config, mission):
    # Read configuration files
    nHaulers, nLP, nULP, nSO, nCS, hauler_positions, LP_positions, ULP_positions, SO_positions, CS_positions, max_energy, initial_energy = config
    mission = mission[0]
    
    unique_missions = {}
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
                
    for id, pos in zip(mission, mission_pos):
        unique_missions[id] = pos
    unique_missions['IH'] = hauler_positions[0]

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
    completion_times = 0
    
    # ----------------------------------------
    # Start path finding
    start = time.perf_counter()
    # Get the mission for each hauler
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
    completion_times = [len(mission_path)]
    
    mission_paths  = paths.copy()
    mission_path = mission_path

    CS_position = CS_positions[0]
    paths = mission_paths
    # ----------------------------------------
    # Find the distance to the charger
    pred = {}
    distance = distance_to_charger(CS_position, unique_missions, grid_map, distance, pred)
    # Find the path from charger to mission points
    charger_paths = {}
    charger_next_paths = {}
    for id, pos in unique_missions.items():
        path =[]
        path = find_path(CS_position, pos, path, pred)
        CS_x, CS_y = CS_position
        charger_paths[id] = [[CS_x, CS_y]] + path
        path.reverse()
        charger_next_paths[id] = path + [[CS_x, CS_y]]
    
        # ----------------------------------------
    # Find the distance to the charger
    pred = {}
    distance = distance_to_charger(CS_position, unique_missions, grid_map, distance, pred)
    # Find the path from charger to mission points
    charger_paths = {}
    charger_next_paths = {}
    for id, pos in unique_missions.items():
        path =[]
        path = find_path(CS_position, pos, path, pred)
        CS_x, CS_y = CS_position
        charger_paths[id] = [[CS_x, CS_y]] + path
        path.reverse()
        charger_next_paths[id] = path + [[CS_x, CS_y]]
    
    # ----------------------------------------
    # Create tables for energy caclulation
    total_energy_cost = len(mission_path) * ENERGY_COST
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
            next_cost.append(len(paths[i]) * ENERGY_COST)
            charger_cost.append(len(charger_paths[id]) * ENERGY_COST)
            charger_next_cost.append(0)
            end_cost.append(prev_energy_cost)
            continue
    
    # ----------------------------------------
    # Find the shortest path with energy
    charger_mission = find_shortest_path_energy(max_energy, initial_energy, total_energy_cost, next_cost, end_cost, charger_cost, charger_next_cost, mission)
    # Create the final path
    # print(f"{charger_mission = }")
    mission_charger_path = []
    if not charger_mission == mission:
        for i,val in enumerate(charger_mission):
            try:
                if val == 'CS':
                    # Add charging time, in total 5 seconds
                    mission_charger_path += charger_next_paths[mission[i-1]][1:]
                    # print(f"Recharging at {mission[i-1]} with {len(mission_charger_path)}")
                    # print(f"{charger_next_paths[mission[i-1]] = }")
                    # print(f"{charger_paths[mission[i]] = }")
                    mission_charger_path += [CS_position]
                    mission_charger_path += [CS_position]
                    mission_charger_path += [CS_position]
                    
                    mission_charger_path += charger_paths[mission[i]]
                else:
                    mission_charger_path += paths[i]
            except:
                break
        final_path = mission_charger_path
    else:
        final_path = mission_path 
    # Caclulate the total distance
    completion_times = len(final_path) - 1
    makespan = completion_times + 1

    # Stop path finding
    end = time.perf_counter()
    execution_time = (end - start)*1000
    print(f"{execution_time = :.2f} ms")
    
        
    write_output(makespan, [completion_times], execution_time, [final_path])


if __name__ == "__main__":
    config = read_config()
    mission = read_mission()
    part2(config, mission)



