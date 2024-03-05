ENERGY_COST = 100

class Tabulation():
    def __init__(self, total_e, init_e, paths, charger_paths, mission, max_energy) -> None:
        self.node2end = [total_e * 2]
        self.start2node = [0]
        self.cap_cons = [init_e]
        self.detour_cost = []
        self.nodes2cs = []
        self.node2next = []
        
        for i,id in enumerate(mission):
            id = id[:2]

            next_cost = (len(paths[i - 1])) * ENERGY_COST
            node2cs = len(charger_paths[id]) * ENERGY_COST
            self.node2next.append(next_cost)
            self.nodes2cs.append(node2cs)
            cs2next = self.nodes2cs[i - 1]
            if id == 'IH':
                self.detour_cost.append(node2cs + cs2next - next_cost)
                continue
            
            self.start2node.append(self.start2node[i-1] + next_cost)
            self.node2end.append(self.node2end[i - 1] - next_cost)
            self.cap_cons.append(self.cap_cons[i - 1] - next_cost)
            self.detour_cost.append(node2cs + cs2next - next_cost)

        self.mission = mission
        self.max_energy = max_energy
        self.paths = paths
        self.charger_paths = charger_paths
        self.min_score = float('inf')
        self.min_mission = []
        self.reach_end = False
        self.print_tabs()

    def print_tabs(self):
        print(f'{self.start2node = }')
        print(f'{self.node2end = }')
        print(f'{self.cap_cons = }')
        print(f'{self.node2next = }')
        print(f'{self.nodes2cs = }')
        print(f'{self.detour_cost = }')
    
    def get_score(self):
        print(f'{self.min_score = }')
        print(f'{self.min_mission = }')
        return self.min_score, self.min_mission

        
    def run(self):
        self.cal_cons(0, self.cap_cons[0], 0, self.mission)
        return

        
    def cal_cons(self, idx, init_cap, score, curr_path):
        cap_cons = [init_cap]
        reached_end = True
        for i in range(1, len(self.mission) - idx):
            next_cost = cap_cons[i] - self.node2next[i]
            cap_cons.append(next_cost)
            if next_cost <= 0:
                reached_end = False
                break

        if reached_end:
            if score < self.min_score:
                self.min_score = min(self.min_score, score)
                self.min_mission = curr_path
            return
        
        for i in range(len(cap_cons) - 1 + idx, idx, -1):
            if self.cap_cons[i] < self.node2cs[i]:
                continue
            cap = self.max_energy - self.node2cs[i + 1]
            score += self.detour_cost[i]
            next_path = curr_path.copy()
            next_path.insert(i + 2, "CS")
            self.cal_cons(i, cap, score, next_path)
        

            # if self.cap_cons[i] < self.nodes2cs[i]:
            #     print(f"Cannot reach CS {self.cap_cons[i] = } {self.nodes2cs[i] = }")
            #     continue
            # cap = self.max_energy - self.nodes2cs[i + 1]
            # print(f'{cap = }')
            # if cap >= self.node2end[i + 1]:
            #     print(f'End in reach {self.node2end[i + 1]} {self.mission[i]}')
            #     self.reach_end = True
            #     score = self.detour_cost[i]

            #     if score < self.min_score:
            #         self.min_score = score
            #         self.min_mission = self.mission.copy()
            #         self.min_mission.insert(i + 2, "CS")
            #         print(f'{self.min_score = }')
        # while(not self.reach_end):
        #     for i in range(ooe - 1, 0, -1):
        #         self.cal_cons(i)
        #     break