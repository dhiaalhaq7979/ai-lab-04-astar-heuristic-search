"""
Lab 4 experiments
  * A* with a full trace (same logic as the manual, I only added logging)
  * UCS as a control (A* with every h = 0)
  * Task 3: what happens when the heuristic overestimates
Counting rule: a node counts as "expanded" only when its successors are generated.
The goal pop and outdated (stale) queue entries are not counted.
"""
import heapq

try:
    from .astar_manual import astar, g as GRAPH, h as H
    from .heuristic_checks import real_cost_to_goal, report
except ImportError:
    from astar_manual import astar, g as GRAPH, h as H
    from heuristic_checks import real_cost_to_goal, report


def astar_with_log(graph, h, start, goal):
    queue = [(h[start], 0, start, [start])]
    best = {start: 0}
    log, expanded, popped = [], [], []
    while queue:
        f, g_val, node, path = heapq.heappop(queue)
        if node == goal:
            popped.append(node)
            log.append(("GOAL", node, g_val, h[node], f, "-", list(sorted(queue))))
            return {"cost": g_val, "path": path, "expanded": expanded, "popped": popped, "log": log}
        if g_val != best[node]:
            log.append(("STALE", node, g_val, h[node], f, "-", list(sorted(queue))))
            continue
        popped.append(node)
        expanded.append(node)
        generated = []
        for nxt, w in graph.get(node, []):
            new_g = g_val + w
            if new_g < best.get(nxt, float("inf")):
                best[nxt] = new_g
                heapq.heappush(queue, (new_g + h[nxt], new_g, nxt, path + [nxt]))
                generated.append(f"{nxt}(g={new_g}, f={new_g}+{h[nxt]}={new_g + h[nxt]})")
            else:
                generated.append(f"{nxt}(g={new_g} not better)")
        log.append(("EXPAND", node, g_val, h[node], f, ", ".join(generated) or "-", list(sorted(queue))))
    return {"cost": None, "path": None, "expanded": expanded, "popped": popped, "log": log}


def frontier_text(queue):
    return " ".join(f"{n}[f={f},g={g}]" for f, g, n, _ in queue) or "empty"


def verify_cost(graph, path):
    weights = [dict(graph[u])[v] for u, v in zip(path, path[1:])]
    return sum(weights), " + ".join(map(str, weights))


def show_run(title, graph, h):
    result = astar_with_log(graph, h, "S", "G")
    print(title)
    print(f"  h = {h}")
    print("  # | action | node | g | h | f | generated | frontier after")
    for i, (action, node, g_val, h_val, f, gen, queue) in enumerate(result["log"], start=1):
        print(f"  {i} | {action:<6} | {node} | {g_val} | {h_val} | {f} | {gen} | {frontier_text(queue)}")
    total, expr = verify_cost(graph, result["path"])
    print(f"  path {'-'.join(result['path'])}, cost {result['cost']} (check: {expr} = {total})")
    print(f"  expanded {len(result['expanded'])}: {' '.join(result['expanded'])} | pop order: {' '.join(result['popped'])}")
    return result


def overestimate_sweep(graph, h, extra=5):
    """Overestimate ONE node at a time by `extra` above its real cost and see if A* breaks."""
    h_star = real_cost_to_goal(graph, "G")
    optimal = astar(graph, h, "S", "G")[0]
    print(f"  Each row: one node gets h = h*(n) + {extra}, everything else unchanged")
    print("  node | h*(n) | new h | path    | cost | still optimal?")
    for node in ("A", "B", "C"):
        test_h = dict(h)
        test_h[node] = h_star[node] + extra
        cost, path = astar(graph, test_h, "S", "G")
        print(f"  {node:<4} | {h_star[node]:<5} | {test_h[node]:<5} | {'-'.join(path):<7} | {cost:<4} | "
              f"{'yes' if cost == optimal else 'NO'}")


def run():
    print("-" * 70)
    print("Lab 4 | A* search, heuristic checks and overestimation")
    print("-" * 70)
    print("graph:", GRAPH)
    print("manual astar(g, h, 'S', 'G') ->", astar(GRAPH, H, "S", "G"))
    print()
    print("Heuristic check (supplied h):")
    report(GRAPH, H)
    print()
    a = show_run("Tasks 1-2 | A* with the supplied heuristic:", GRAPH, H)
    print()
    u = show_run("Task 2 | UCS control (h = 0 everywhere, same code):", GRAPH, {n: 0 for n in H})
    print()
    bad_h = dict(H)
    bad_h["C"] = 10
    print("Task 3 | Overestimate: h(C) = 10 while the real remaining cost from C is 3")
    report(GRAPH, bad_h)
    o = show_run("A* with h(C) = 10:", GRAPH, bad_h)
    print()
    print("Summary:")
    print("  run           | path    | cost | expansions")
    for name, r in (("A* (given h)", a), ("UCS (h = 0)", u), ("A* h(C) = 10", o)):
        print(f"  {name:<13} | {'-'.join(r['path']):<7} | {r['cost']:<4} | {len(r['expanded'])}")
    print()
    print("Extra | overestimating each node separately:")
    overestimate_sweep(GRAPH, H)
    print("  -> overestimating B does not hurt here, but A or C does: a run that 'looks fine'")
    print("     does not prove that an inadmissible heuristic is safe.")


if __name__ == "__main__":
    run()
