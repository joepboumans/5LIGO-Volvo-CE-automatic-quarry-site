#!/usr/bin/env python3
"""
A tool to check output files of assignment 1
    Copyright © 2021 Pourya Gohari

Usage:
    sanity_check.py [-c FILE] [-m FILE] [-o FILE]
    sanity_check.py -h

Options:
    -c FILE, --config FILE       config file [default: config.txt]
    -m FILE, --mission FILE      mission file [default: mission.txt]
    -o FILE, --output FILE       output file [default: output.txt]
    -h, --help                   show this message
"""


from docopt import docopt
import re
import os.path


def read_config(file="config.txt"):
    f = open(file, "r")
    lines = f.readlines()

    # for following the line number
    index = 1
    n_Hauler = 1
    n_LP = 0
    n_ULP = 0
    n_SO = 0
    n_CS = 1
    init_haulers = []
    LP_positions = []
    ULP_positions = []
    SO_positions = []
    CS_positions = []
    batery_capacity = 0
    initial_energy = 0

    for line in lines:
        # read number of Haulers
        if(index == 1):
            n_Hauler = int(line.split("//")[0])

        # read number of LPs
        elif(index == 2):
            n_LP = int(line.split("//")[0])

        # read number of ULPs
        elif(index == 3):
            n_ULP = int(line.split("//")[0])

        # read number of SOs
        elif(index == 4):
            n_SO = int(line.split("//")[0])

        # read number of CSs
        elif(index == 5):
            n_CS = int(line.split("//")[0])

        # read Initial position of haulers
        elif(index == 6):
            t = line.split("//")[0].split("-")
            for i in range(n_Hauler):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                init_haulers.append(x)

        # read position of LPs
        elif (index == 7):
            t = line.split("//")[0].split("-")
            for i in range(n_LP):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                LP_positions.append(x)

        # read position of ULPs
        elif (index == 8):
            t = line.split("//")[0].split("-")
            for i in range(n_ULP):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                ULP_positions.append(x)

        # read position of SOs
        elif (index == 9):
            t = line.split("//")[0].split("-")
            for i in range(n_SO):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                SO_positions.append(x)
        # read position of CSs
        elif (index == 10):
            t = line.split("//")[0].split("-")
            for i in range(n_CS):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                CS_positions.append(x)
        # read battery capacity
        elif(index == 11):
            if(line.split("//")[0] != '\t'):
                batery_capacity = int(line.split("//")[0])

        # read initial energy
        elif(index == 12):
            if(line.split("//")[0] != '\t'):
                initial_energy = int(line.split("//")[0])
        index += 1
    return init_haulers, LP_positions, ULP_positions, SO_positions, CS_positions, batery_capacity, initial_energy


def read_mission(file="mission.txt"):
    f = open(file, "r")
    lines = f.readlines()

    mission = []

    for line in lines:
        temp = line.split("\t//")[0].split(",")
        mission.append(temp)

    return mission


def read_output(file="output.txt"):
    f = open(file, "r")
    lines = f.readlines()

    vertex_template = "\[(\d+),(\d+)\]"
    vertex_template2 = "\[(\d+), (\d+)\]"

    makespan = 0
    completion_time = []
    exec_time = 0

    print(lines)
    for line in lines:
        if ("Makespan" in line):
            match = re.search('(\d+)\t//Makespan', line)
            makespan = int(match.group(1))
            print(match.group(1))
        if("Mission completion time" in line):
            match = re.search(
                '(\d+)\t//Mission completion time hauler (\d+)', line)
            completion_time.append(int(match.group(1)))
            print (completion_time)

        if("Application execution time" in line):
            match = re.search(
                    '([-+]?\d*\.\d+|\d+)\t//Application execution time', line)
            print(match.group(1))
            exec_time = match.group(1)[:5]


    path_template = "(\d+),"
    path_template2 = "(\d+),"

    for i in range(len(completion_time)):

        if(i == len(completion_time)-1):
            path_template += vertex_template
            path_template2 += vertex_template2
        else:
            path_template += vertex_template+","
            path_template2 += vertex_template2+","

    positions = [[]for i in range(len(completion_time))]
    for line in lines:

        if(re.search(path_template, line)):
            # temp = line.split(",")
            match = re.search(path_template, line)
            # print (match.group(5))
            for i in range(len(completion_time)):
                positions[i].append(
                    [int(match.group((i*2)+2)), int(match.group((i*2)+3))])
        elif(re.search(path_template2, line)):
            #temp = line.split(",")
            match = re.search(path_template2, line)
            #print (match)
            for i in range(len(completion_time)):
                positions[i].append(
                    [int(match.group((i*2)+2)), int(match.group((i*2)+3))])
                    
    return makespan, completion_time, exec_time, positions


def convert_mission(missions, LP_positions, ULP_positions):
    missions_position = []
    for mission in missions:
        x = []
        for m in mission:
            if(re.search('L(\d+)', m)):
                match = re.search('L(\d+)', m)
                index = int(match.group(1))-1
                x.append(LP_positions[index])
            elif(re.search('U(\d+)', m)):
                match = re.search('U(\d+)', m)
                index = int(match.group(1))-1
                x.append(ULP_positions[index])

        missions_position.append(x)
    # print (missions_position)
    return missions_position


def check_init(init_haulers, positions):
    for i in range(len(init_haulers)):
        if(init_haulers[i] != positions[i][0]):
            print("Initial point " + str(positions[i][0])+" is not correct.")
            return False
    return True


def check_moves(positions):
    for hauler in positions:
        for i in range(len(hauler)):
            if(i != len(hauler)-1):
                # Check for possible jump in the movements
                if(abs(hauler[i][0]-hauler[i+1][0])+abs(hauler[i][1]-hauler[i+1][1]) > 1):
                    print(hauler[i], hauler[i+1])
                    return False
    return True


def check_makespan(makespan, positions):
    max = 0
    # Just in case that was a inconsisty between two haulers positions
    for hauler in positions:
        temp = len(hauler)
        print(temp);
        # print(hauler[temp-1]);
        
        max = temp if temp > max else max
    # print (max)
    if (makespan == max or makespan == max - 1 or makespan == max + 1):
        return True
    else:
        return False


def check_obstacles(SO_positions, positions):
    for so in SO_positions:
        for hauler in positions:
            if(so in hauler):
                print("Obstacles collision in "+str(so)+".")
                return False
    return True


def check_haulers_collision(positions, completion_time):
    for i in range(len(positions[0])):
        for j in range(len(positions)):
            # if mission of this hauler is completed we don't consider it
            if(completion_time[j] < i):
                continue
            for k in range(j+1, len(positions)):
                # print (str(i) + str(positions[j][i])+"\t" +str(positions[k][i]))
                # if mission of this hauler is completed we don't consider it
                if(completion_time[k] < i):
                    continue
                if(positions[j][i] == positions[k][i]):
                    print("Haulers collision in time "+str(i)+".")
                    return False

    return True


def check_mission(missions, positions):
    for i, hauler in enumerate(positions):
        x=0
        
        for m in missions[i]:
            if(not m in hauler):
                print("Mission "+str(m)+" failed.")
                return False
            else:
                find_flag = False
                for j in range(x,(len(hauler))):
                    if(hauler[j] == m):
                        x= j
                        find_flag = True
                        #print("Hauler "+ str(i)+ " "+ "mission "+ str(m) + " meets in step "+ str(x))
                        break
                if(not find_flag):
                    print("Mission "+str(m)+" failed.")
                    return False
    return True

def check_charging(positions, completion_time,batery_capacity,initial_energy,CS_positions):
    for i in range(len(positions)):
        cur_battery=initial_energy
        j=0
        while j < completion_time[i]-1:
            print(str(j) +"\t" +str(cur_battery))
            if(positions[i][j] in CS_positions and j+5 < completion_time[i]):
                index = CS_positions.index(positions[i][j])
                wait =0
                
                for k in range(j, j+5):
                    if(positions[i][k] == CS_positions[index]):
                        wait +=1
                if(wait == 5):
                    print ("..CHARGED..")
                    j+=5
                    cur_battery = batery_capacity
            if(cur_battery <= 0):
                print ("Out of charge in " + str(positions[i][j]) + f" {j}")
                return False
            cur_battery -= 50
            j+=1
    return True


def write_result(text):
    f = open("result.txt", "w")
    f.write(text)
    f.close()



def main():
    arguments = docopt(__doc__, version='0.9.4')
    config_file = arguments['--config']
    mission_file = arguments['--mission']
    output_file = arguments['--output']
    if(os.path.exists(output_file)):
        init_haulers, LP_positions, ULP_positions, SO_positions, CS_positions, batery_capacity, initial_energy = read_config(
            config_file)

        mission = read_mission(mission_file)
        makespan, completion_time, exec_time, positions = read_output(output_file)

        initial_test = move_test = makespan_test = obstacle_test = hauler_test = mission_test = charging_test = True

        if(not check_init(init_haulers, positions)):
            print("Initial point of haulers are not correct!")
            initial_test = False
        else:
            print("Initial point of haulers checked successfully!")

        if(not check_moves(positions)):
            print("Illigal move found in output file!")
            move_test = False
        else:
            print("Movements checked successfully!")

        if(not check_makespan(makespan, positions)):
            print("Calculated makespan is wrong!")
            makespan_test = False
        else:
            print("Makespan checked successfully!")

        if(not check_obstacles(SO_positions, positions)):
            print("Obstacles collision detected!")
            obstacle_test = False
        else:
            print("Obstacles checked successfully!")

        if(not check_haulers_collision(positions, completion_time)):
            print("Haulers collision detected!")
            hauler_test = False
        else:
            print("Haulers checked successfully!")


        if(len(CS_positions) != 0):
            try:
                if(not check_charging(positions, completion_time,batery_capacity,initial_energy,CS_positions)):
                    print("Hauler out of charge detected!")
                    charging_test = False
                else:
                    print("Haulers charge checked successfully!")
            except:
                print("error in chcking charge!")
                charging_test = False

        missions_position = convert_mission(mission, LP_positions, ULP_positions)

        if(not check_mission(missions_position, positions)):
            print("Mission goals are not met!")
            mission_test = False
        else:
            print("Mission goals checked successfully!")
    else:
        initial_test = move_test = makespan_test = obstacle_test = hauler_test = mission_test = charging_test = False
        makespan= completion_time = exec_time = -1

    result_text = str(makespan)+"\n"+str(exec_time)[:10]+"\n" + str(initial_test)+"\n" + str(move_test)+"\n" + str(makespan_test) + \
        "\n" + str(obstacle_test)+"\n" + str(hauler_test) + \
        "\n" + str(charging_test)+"\n" + str(mission_test)

    write_result(result_text)


if __name__ == '__main__':
    main()
