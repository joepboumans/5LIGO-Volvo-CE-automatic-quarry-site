import re
import shutil
import os
from glob import glob
import csv

from part1 import part1
from part2 import part2
from part3 import part3


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


if __name__ == "__main__":
    files = [y for x in os.walk('Part2') for y in glob(os.path.join(x[0], '*.txt'))]
    files = [f.replace("config.txt", '').replace('mission.txt', '') for f in files]
    files = list(set(files))
    files.sort()

    baseline_makespan = {}
    with open('baseline.csv') as baseline:
        reader = csv.reader(baseline)
        curr_part = ''
        curr_diff = ''
        curr_length = ''
        for row in reader:
            if "Part" in row[0]:
                curr_part = row[0]
                continue
            if "-" in row[0]:
                curr_diff = row[0]
                continue
            if "Mission" in row[0]:
                mission_length = re.findall(r'(\d+)', row[0])[0]
                curr_length = mission_length
                continue
            curr_mission = curr_part + curr_diff + curr_length + row[0]
            curr_mission.strip('\n')
            baseline_makespan[curr_mission] = int(row[1])
                
    for f in files:
        input_path = f
        # input_path = "Part2\\1-Medium\\12\\5_"
        if input_path != "":
            shutil.copyfile(input_path + "mission.txt", "mission.txt")
            shutil.copyfile(input_path + "config.txt", "config.txt")
        # Read configuration files
        nHaulers, nLP, nULP, nSO, nCS, hauler_positions, LP_positions, ULP_positions, SO_positions, CS_positions, max_energy, initial_energy = read_config()
        config = nHaulers, nLP, nULP, nSO, nCS, hauler_positions, LP_positions, ULP_positions, SO_positions, CS_positions, max_energy, initial_energy
        mission = read_mission()

        nHaulers = int(nHaulers)
        nCS = int(nCS)
        if nHaulers == 1 and nCS == 0:
            print("Part 1")
            part1(config, mission)
        elif nHaulers == 1 and nCS == 1:
            print("Part 2")
            part2(config, mission)
        else:
            print("Part 3")
            part3(config, mission)

        import sanity_check
        sanity_check.main()

        with open('result.txt') as res:
            lines = res.readlines()
            makespan = int(lines[0])

        name = input_path.replace('\\', '').replace('_','')
        baseline_span = baseline_makespan[name]

        print(f"{baseline_span = } vs {makespan = }")
        if baseline_span < makespan:
            print("NOT OPTIMAL SOLUTION")
            print(input_path)
            exit()