"""
Heuristic quality checks
  admissible : h(n) <= h*(n)               (never overestimates the real remaining cost)
  consistent : h(u) <= cost(u, v) + h(v)   (for every edge u -> v)
h*(n) is calculated by running A* with h = 0 (which is just UCS) from every node.
"""
try:
    from .astar_manual import astar
except ImportError:
    from astar_manual import astar


def real_cost_to_goal(graph, goal):
    no_info = {node: 0 for node in graph}
    table = {}
    for node in graph:
        answer = astar(graph, no_info, node, goal)
        table[node] = answer[0] if answer else float("inf")
    return table


def is_admissible(graph, h, goal):
    h_star = real_cost_to_goal(graph, goal)
    rows = [(n, h[n], h_star[n], h[n] <= h_star[n]) for n in graph]
    return all(ok for *_, ok in rows), rows


def is_consistent(graph, h):
    rows = []
    for u, edges in graph.items():
        for v, cost in edges:
            rows.append((u, v, h[u], cost, h[v], h[u] <= cost + h[v]))
    return all(r[-1] for r in rows), rows


def report(graph, h, goal="G"):
    ok_a, rows_a = is_admissible(graph, h, goal)
    print("  Admissibility  h(n) <= h*(n)")
    print("    node | h(n) | h*(n) | ok?")
    for n, hn, hs, ok in rows_a:
        print(f"    {n:<4} | {hn:<4} | {hs:<5} | {'yes' if ok else 'NO - overestimates'}")
    ok_c, rows_c = is_consistent(graph, h)
    print("  Consistency  h(u) <= c(u,v) + h(v)")
    print("    edge  | h(u) | c + h(v)      | ok?")
    for u, v, hu, c, hv, ok in rows_c:
        print(f"    {u}->{v}  | {hu:<4} | {c} + {hv} = {c + hv:<4} | {'yes' if ok else 'NO'}")
    print(f"  => admissible: {ok_a}, consistent: {ok_c}")
    return ok_a, ok_c
