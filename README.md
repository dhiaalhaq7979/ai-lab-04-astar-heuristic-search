# AI Lab 04 — A* Search and Heuristic Design

![Python](https://img.shields.io/badge/Python-3.x-3776AB?style=flat-square&logo=python&logoColor=white)
![Dependencies](https://img.shields.io/badge/dependencies-none-success?style=flat-square)
![License](https://img.shields.io/badge/license-MIT-blue?style=flat-square)

In this lab I moved from uninformed to **informed search**. A* ranks every path by

```
f(n) = g(n) + h(n)
```

where `g(n)` is the cost so far and `h(n)` is a heuristic estimate of the cost left to the goal. I traced A* step by step, checked whether the heuristic is **admissible** and **consistent**, used UCS as a control, and then tested what happens when the heuristic **overestimates**.

## Structure

```
ai-lab-04-astar-heuristic-search/
├── run_lab.py
├── informed/
│   ├── astar_manual.py        # A*, graph and heuristic from the manual
│   ├── heuristic_checks.py    # admissibility + consistency checks
│   └── experiments.py         # traced A*, UCS control, overestimate tests
└── results/
    └── sample_run.txt
```

## Run

```bash
python run_lab.py
```

## The problem

| Edge | S→A | S→B | A→C | B→G | C→G |
|------|-----|-----|-----|-----|-----|
| Cost | 1 | 4 | 2 | 5 | 3 |

Heuristic: `h = {S: 5, A: 4, B: 4, C: 2, G: 0}` — my checks show it is both **admissible** and **consistent**.

## A* trace (supplied heuristic)

| Step | Expand | g | h | f | Frontier after |
|------|--------|---|---|---|----------------|
| 1 | S | 0 | 5 | 5 | A (f=5), B (f=8) |
| 2 | A | 1 | 4 | 5 | C (f=5), B (f=8) |
| 3 | C | 3 | 2 | 5 | G (f=6), B (f=8) |
| 4 | G | 6 | 0 | 6 | goal reached |

## Results

| Run | Path | Cost | Expansions* |
|-----|------|------|-------------|
| A* (given h) | S-A-C-G | **6** | 3 |
| UCS (h = 0) | S-A-C-G | 6 | 4 |
| A* with h(C) = 10 | S-B-G | 9 ✘ | 3 |

\* A node is counted as expanded only when its successors are generated (the goal pop is not counted).

- A* and UCS find the same optimal path, but **A* expands fewer nodes** — the heuristic saves work.
- When `h(C) = 10` (the real remaining cost is only 3), A* is pushed away from C and returns **S-B-G with cost 9**, which is not optimal.

## Extra — overestimating one node at a time

| Node overestimated | New h | Path | Cost | Optimal? |
|--------------------|-------|------|------|----------|
| A | 10 | S-B-G | 9 | No |
| B | 10 | S-A-C-G | 6 | Yes |
| C | 8 | S-B-G | 9 | No |

Overestimating B happens to change nothing, but A or C breaks optimality. So one run that looks correct does **not** prove that an inadmissible heuristic is safe.

## Author

**Dhia Alhaq Saleh Al-Hindrish** — BS Software Engineering, Superior University, Lahore
GitHub: [@dhiaalhaq7979](https://github.com/dhiaalhaq7979)

Licensed under the [MIT License](LICENSE).
