# Random-maze-generation

This is a visualizer for three different algorithms involving graphs. They are all built upon a single graph as a grid of vertices and edges of random weight connecting each adjacent vertex. These algorithms are:
## a) Random maze generator (dijkstra)
A first attempt at making a random maze. It uses dijkstra's algorithm (breadth first search that prioritizes edges of lower weight) to find a minimum spanning tree of the graph, which is the resulting "labyrinth". Press 'D' to begin the algorithm



## b) Random maze generator (depth first search)
This is an improved version of the previous algorithm. It is a depth first search that moves randomly. Its speed can be changed by changing DFS_DELAY in line 22 or it can be made instantaneous by commenting and uncommenting the lines I marked. Press 'F' to begin the algorithm.

![dfs gif](https://github.com/MiniLink80/Path-And-Maze-Algorithms/blob/main/demos/dfs)

## c) Self avoiding walk generator
This algorithm starts from the top left of the graph (or at a random vertex by uncommenting the relevant line in UniquePath()) and ends when every vertex in the graph has been visited only once. It works by assigning a stack to every vertex. These stacks contain all valid adjacent vertices we can move to from any given vertex. Even if the stack of a vertex gets depleted, it will be reset when we reach that vertex from a different path, so the algorithms time complexity increases incredibly fast (O(4<sup>height*width</sup>), I believe). That's why I used two optimization methods. Namely, an island counter and a cul de sac counter. The algorithm starts backtracking as soon as either of these is above 1. Press 'G' to begin the algorithm. (Keep in mind that it might either take a VERY long time or get done instantly so don't increase the grid size too much). By default it will update its animation every 10000 iterations but you can change that in line 23.

### Controls:
- D: Algorithm a)
- F: Algorithm b)
- G: Algorimth c)
- Esc: Quit (will not work during an animation so be extra careful in alg c)
- Modify RECT_SIZE in line 21 to change grid size
