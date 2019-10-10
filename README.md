#Tree Search Optimazaion

In corporate DFS, BFS, uniformed-cost and A* Tree search algorithms and improved the overall run time for searches 
with heuristics by around 50% to 60% by changing the heuristics.

#### Heuristic Method

In sample_heuristics, it used euclidean's distance and initialized all points' distance to finish line.
This method is very inefficient. Therefore, revised heuristic runs faster by using diagonal distance with certain 
constraints such as direction and speed at each state.

### Run Program
```bash
python3 start.py
```
This command will generate a problem with visualization. Changing parameters inside proj1.py will generate different problems
and different searching methods. 
