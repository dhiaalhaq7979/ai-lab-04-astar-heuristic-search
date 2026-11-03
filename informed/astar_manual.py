# A* implementation, graph and heuristic exactly as given in the lab manual (section 4.2).
# Expected output: (6, ['S', 'A', 'C', 'G'])
import heapq
def astar(graph,h,start,goal):
    pq=[(h[start],0,start,[start])]; best={start:0}
    while pq:
        f,g,node,path=heapq.heappop(pq)
        if node==goal: return g,path
        if g!=best[node]: continue
        for nxt,w in graph.get(node,[]):
            ng=g+w
            if ng < best.get(nxt,float("inf")):
                best[nxt]=ng
                heapq.heappush(pq,(ng+h[nxt],ng,nxt,path+[nxt]))

g={"S":[("A",1),("B",4)],"A":[("C",2)],"B":[("G",5)],"C":[("G",3)],"G":[]}
h={"S":5,"A":4,"B":4,"C":2,"G":0}

if __name__ == "__main__":
    print(astar(g,h,"S","G"))
