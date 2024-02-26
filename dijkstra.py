class Graph():
    def __init__(self, adjecents) -> None:
        self.size_adj = len(adjecents)
        self.adj = adjecents
        self.graph = [0 for _ in range(self.size_adj * 2)]
    
    def dijkstra(self, start):
        dist = [float('inf')] * self.size_adj
        dist[start] = 0
        vistited = [False] * self.size_adj