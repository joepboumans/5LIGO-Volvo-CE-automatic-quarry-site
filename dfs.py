CS_TIME = 5
ENERGY_COST = 100
class DFS():
    def __init__(self, max_cap, adj_list, end_node) -> None:
        self.max_cap = max_cap
        self.adj_list = adj_list
        self.min_score = float('inf')
        self.min_path = []
        self.end_node = end_node
        self.iterations = 0
    
    def run(self, node, next_cost, cap, path, score):
        self.iterations += 1

        score, cap = self.at_CS(node, score, cap)
        if cap < next_cost * ENERGY_COST:
            print(f'Cannot reach next {next_cost = }, {cap = }, {node = }')
            return (path, float('inf'))
        
        path.append(node)
        score += next_cost
        cap -= next_cost * ENERGY_COST

        if score > self.min_score:
            return (path, float('inf'))
        if node == self.end_node:
            if score < self.min_score:
                self.min_score = score
                self.min_path = path.copy()
                print(f'Smaller scored path found! {self.min_score = }\n {self.min_path = }\n{cap = }')
            return (path, score)
        
        print(f'{path = }\n:\t{node = }')
        adj_paths = []
        for adj in self.adj_list[node]:
            adj_paths.append(self.run(adj[0], adj[1], cap, path, score))
        
        # print(adj_paths)
        adj_score = {adj[1]:adj[0] for adj in adj_paths}
        min_adj = min(adj_score)
        return (adj_score[min_adj], min_adj)
    
    def at_CS(self, node, score, cap):
        if 'CS' in node:
            cap = self.max_cap
            score += CS_TIME
        return score, cap


mission = ['L1', 'U1', 'L3', 'U1_2', 'L2']
next_cost = [10, 8, 20, 15]
node2cs_cost = [3, 6, 15, 6, 2]
cs2next_cost = [4, 3, 15, 4]
cs_time = 5
init_cap = 3350
max_cap = 4730

adj_list = {}
for i,m in enumerate(mission):
    if m == mission[-1]:
        continue
    adj_list[m] = [(mission[i + 1], next_cost[i]),('CS_' + m, node2cs_cost[i] + cs_time)]
    adj_list['CS_' + m] = [(mission[i + 1], cs2next_cost[i])]

print(adj_list)
print(f'Finding path from {mission[0]} to {mission[-1]}')
# dfs(mission[-1], mission[0], next_cost[0], path, 0)

dfs = DFS(max_cap, adj_list, mission[-1])
dfs.run(mission[0], next_cost[0], init_cap, [], 0)
print('\n\n')
print(f'Final path {dfs.min_path}\nWith {dfs.min_score = }')
print(f'{dfs.iterations = }')