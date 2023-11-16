import re
import numpy as np

GRID_SIZE = 12
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


def bfs(start, end, grid_map, distance):
    # Unpack start and end
    start_x, start_y = start
    end_x, end_y = end
    
    # BFS
    visited = np.full((GRID_SIZE, GRID_SIZE), False)
    queue = []
    bfs = []
    
    # Start from Hauler position
    queue.append((start_x-1, start_y-1))
    visited[start_x-1, (start_y-1)] = True
    
    while queue:
        s = queue.pop(0)
        bfs.append(s)
        if s == (end_x-1, end_y-1):
            break
        x, y = s
        if x > 0 and grid_map[x-1, y] != WALL and not visited[x-1,y]:
            queue.append((x-1, y))
            visited[x-1,y] = True
            distance[x-1,y] = distance[x,y] + 1
        if x < GRID_SIZE-1 and grid_map[x+1, y] != WALL and not visited[x+1,y]:
            queue.append((x+1, y))
            visited[x+1,y] = True
            distance[x+1,y] = distance[x,y] + 1
        if y > 0 and grid_map[x, y-1] != WALL and not visited[x,y-1]:
            queue.append((x, y-1))
            visited[x,y-1] = True
            distance[x,y-1] = distance[x,y] + 1
        if y < GRID_SIZE-1 and grid_map[x, y+1] != WALL and not visited[x,y+1]:
            queue.append((x, y+1))
            visited[x,y+1] = True
            distance[x,y+1] = distance[x,y] + 1
            
    return distance

if __name__ == "__main__":
    nHaulers, nLP, nULP, nSO, nCS, init_haulers, LP_positions, ULP_positions, SO_positions, CS_positions, max_energy, initial_energy = read_config()
    missions = read_mission()
    print(missions)
    # Create adjecency matrix
    grid_map = np.zeros((GRID_SIZE, GRID_SIZE))
    distance = np.zeros((GRID_SIZE, GRID_SIZE))
    
    # Add SO
    for so in SO_positions:
        grid_map[so[0]-1, so[1]-1] = WALL
        distance[so[0]-1, so[1]-1] = WALL
    
    for mission in missions:
        for next_pos in mission:
            bfs(init_haulers[0], LP_positions[int(next_pos[1])-1], grid_map, distance)
    
    print(distance)
    print(grid_map)