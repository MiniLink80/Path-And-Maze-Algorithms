#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <queue>
#include <stack>
#include <algorithm>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include "graph.h"

using std::cout;
using std::endl;
using std::vector;
using std::priority_queue;
using std::stack;
using std::queue;

const int SCREEN_WIDTH = 600;
const int SCREEN_HEIGHT = 600;
const int RECT_SIZE = 35;           //CHANGE THIS TO MODIFY THE SIZE OF THE GRID
const int DFS_DELAY = 10;           //CHANGE THIS TO MODIFY THE SPEED OF DEPTH FIRST SEARCH
const int SAW_ITERUPDATE = 10000;   //CHANGE THIS TO MODIFY RATE OF UPDATE OF SELF AVOIDING WALK

//The height and width in terms of vertices.
const int h = (int)((SCREEN_HEIGHT/RECT_SIZE)/2);
const int w = (int)((SCREEN_WIDTH/RECT_SIZE)/2);

SDL_Window *window = NULL;
SDL_Surface *screenSurface = NULL;
SDL_Renderer *render = NULL;

vector<Node> nodes;
vector<Edge> edges;

std::default_random_engine rng;

/*
UTILITY FUNCTIONS
*/

//Returns the index of the edge connecting the two vertices whose indeces are given
int getEdge(int from, int to){
    if (from == -1 || to == -1) 
        return -1;
    if (from >= nodes.size() || to >= nodes.size())
        return -1;
    
    for (auto &edge : nodes[from].edges){
        if (edges[edge].from == from && edges[edge].to == to){
            return edge;
        }
        if (edges[edge].from == to && edges[edge].to == from){
            return edge;
        }
    }
    
    return -1;
}

void createEdgesAndVertices(vector<Node> &nodes, vector<Edge> &edges){
    nodes.clear();
    edges.clear();
    
    //Creating a grid of vertices
    for (int i = 0; i < h; i++){
        for (int j = 0; j < w; j++){
            Node newNode;
            newNode.y = i*RECT_SIZE*2;
            newNode.x = j*RECT_SIZE*2;
            nodes.push_back(newNode);
        }
    }

    //Creating edges between each orthogonal vertex
    //We go through each vertex and add two edges;
    //One connecting to the vertex above it and another to the one to its right
    for (int i = 0; i < h; i++){
        for (int j = 0; j < w; j++){
            if (i != 0){
                Edge newEdge;
                newEdge.weight = rand()%(rand()%255+17);
                newEdge.from = i*w + j;
                newEdge.to = (i-1)*w + j;
                newEdge.y = i*RECT_SIZE*2 - RECT_SIZE;
                newEdge.x = j*RECT_SIZE*2;
                edges.push_back(newEdge);
                nodes[newEdge.from].edges.push_back(edges.size()-1);
                nodes[newEdge.to].edges.push_back(edges.size()-1);
            }
            if (j != w-1){
                Edge newEdge;
                newEdge.weight = rand()%(rand()%255+17);
                newEdge.from = i*w + j;
                newEdge.to = i*w + j + 1;
                newEdge.y = i*RECT_SIZE*2;
                newEdge.x = j*RECT_SIZE*2 + RECT_SIZE;
                edges.push_back(newEdge);
                nodes[newEdge.from].edges.push_back(edges.size()-1);
                nodes[newEdge.to].edges.push_back(edges.size()-1);
            }
        }
    }
}

vector<int> getAdjacent(int u){
    vector<int> res;
    if (u > w-1)        //Top
        res.push_back(u-w);
    if (u%w != 0)       //Left
        res.push_back(u-1);
    if (u%w != w-1)     //Right
        res.push_back(u+1);
    if (u < (h-1)*w)    //Bottom
        res.push_back(u+w);
    
    return res;
}

//Comparation function for edgeIndex priority queue
class CompareEdge{
    public:
        bool operator()(int a, int b){
            return edges[a].weight < edges[b].weight;
        }
};

void drawEdge(int i, bool remove){
    SDL_Rect rect;
    rect.x = edges[i].x;
    rect.y = edges[i].y;
    rect.w = rect.h = RECT_SIZE;
    if (remove) SDL_SetRenderDrawColor(render, 10,10,10,255);
    else        SDL_SetRenderDrawColor(render, 75,75,75,255);

    SDL_RenderFillRect(render, &rect);
}

void drawNode(int i, bool remove, int from){
    SDL_Rect rect;
    rect.x = nodes[i].x;
    rect.y = nodes[i].y;
    rect.h = rect.w = RECT_SIZE;

    if (remove){
        SDL_SetRenderDrawColor(render, 10, 10, 10, 255);
    }
    else {
        SDL_SetRenderDrawColor(render, 75, 75, 75, 255);
        if (i == from) {
            SDL_SetRenderDrawColor(render, 255, 75, 75, 255);
        }
    }
    SDL_RenderFillRect(render, &rect);
    
}

/*
OPTIMIZATION FUNCTIONS
*/

void countIslandsRecur(vector<bool> &M, int i, int j)
{
    //Base condition
    //if i less than 0 or j less than 0 or i greater than ROW-1 or j greater than COL-  or if M[i][j] != 1 then we will simply return
    if (i < 0 || j < 0 || i > (h - 1) || j > (w - 1) || M[i*w+j] != false)
    {
        return;
    }
 
    if (!M[i*w+j])
    {
        M[i*w+j] = true;
        countIslandsRecur(M, i + 1, j);     //right side traversal
        countIslandsRecur(M, i - 1, j);     //left side traversal
        countIslandsRecur(M, i, j + 1);     //upward side traversal
        countIslandsRecur(M, i, j - 1);     //downward side traversal
    }
}
 
int countIslands(vector<bool> &M)
{
    int count = 0;
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            if (!M[i*w+j])
            {
                M[i*w+j] = true;
                count++;
                countIslandsRecur(M, i + 1, j);     //right side traversal
                countIslandsRecur(M, i - 1, j);     //left side traversal
                countIslandsRecur(M, i, j + 1);     //upward side traversal
                countIslandsRecur(M, i, j - 1);     //downward side traversal
            }
        }
    }
    return count;
}

bool checkIfWall(int y, int x, vector<bool> M){
    
    if (x < 0 || x >= w || y < 0 || y >= h){
        return true;
    }
    return M[y*w + x];
}

int countCulsDeSac(vector<bool> M, int pt){
    int c = 0;
    for (int i = 0; i < h; i++){
        for (int j = 0; j < w; j++){
            if (!M[i*w + j] && !(i*w+j-1 == pt || i*w+j+1 == pt || (i-1)*w+j == pt || (i+1)*w+j == pt || i*w+j == pt)){
                int numAdjWalls = 0;
                numAdjWalls += (int)checkIfWall(i,j+1,M);
                numAdjWalls += (int)checkIfWall(i,j-1,M);
                numAdjWalls += (int)checkIfWall(i+1,j,M);
                numAdjWalls += (int)checkIfWall(i-1,j,M);
                if (numAdjWalls >= 3){
                    c++;
                }
            }
        }
    }
    return c;    
}

/*
ALGORITHMS
*/


//Self avoiding walk
void UniquePath(){
    SDL_SetRenderDrawColor(render, 10,10,10,255);
    SDL_RenderClear(render);
    SDL_RenderPresent(render);

    std::ofstream f;
    f.open("out.txt");
    

    createEdgesAndVertices(nodes, edges);

    int from = 0;   //Top left
    //int from = rand()%(h*w);

    vector<int> previous(nodes.size(), -1);
    vector<bool> visited(nodes.size(), false);
    vector<stack<int> > s(nodes.size());
    vector<int> edgesToBeRendered(edges.size(), 0);
    vector<int> nodesToBeRendered(nodes.size(), 0);

    int pt; //Pointer to the current vertex
    int visitedCount = 1; //Loop will stop once every vertex has been visited (once)
    int iter = 0;
    pt = from;
    nodesToBeRendered[from] = 1;

    bool backtracking = false;
    while (visitedCount < visited.size()){
        //SDL_Delay(10);

        visited[pt] = true;
        if (!backtracking){
            while(!s[pt].empty()){
                s[pt].pop();
            }
            vector<int> tempAdj = getAdjacent(pt);
            while(!tempAdj.empty()){
                int r = rand()%tempAdj.size();
                if (!visited[tempAdj[r]] && tempAdj[r] != previous[pt]){
                    s[pt].push(tempAdj[r]);
                }
                tempAdj.erase(tempAdj.begin()+r);
            }
        }
        
        //log every single move and the state of all arrays into a file

        /*
        f << "\n\niteration " << iter << "\npt: " << pt << "\nvisitedCount: " << visitedCount << "\n";
        f << "previous array: ";
        for (int i = 0; i < nodes.size(); i++){
            f <<previous[i] << " ";
        }
        f << "\nvisited array:";
        for (int i=0; i < nodes.size(); i++){
            f << (visited[i] ? " true" : " false");
        }
        f << "\nstack:\n";
        for (int i=0; i < nodes.size(); i++){
            stack<int> aux;
            f << "s " << i << ": ";
            while(!s[i].empty()){
                f << s[i].top() << " ";
                aux.push(s[i].top());
                s[i].pop();
            }
            f << "\n";
            while (!aux.empty()){
                s[i].push(aux.top());
                aux.pop();
            }
        }
        */

        int u;
        bool foundValidAdjacent = false;
        if (!s[pt].empty()){
            u = s[pt].top();
            s[pt].pop();
            
            foundValidAdjacent = true;
        }

        vector<bool> copy = visited;
        int edgeId;
        if (!foundValidAdjacent || countIslands(copy) > 1 || countCulsDeSac(visited, pt) > 1){ //Backtrack
            edgeId = getEdge(pt, previous[pt]);    
            edgesToBeRendered[edgeId] = 0;
            nodesToBeRendered[pt] = 0;
            visited[pt] = false;
            int temp = previous[pt];
            previous[pt] = -1;
            pt = temp;
            visitedCount--;
            backtracking = true;
        }
        else { //Keep walking
            edgeId = getEdge(pt, u);
            edgesToBeRendered[getEdge(pt, u)] = 1;
            nodesToBeRendered[u] = 1;
            previous[u] = pt;
            pt = u;
            visitedCount++;
            backtracking = false;
        }
        
        //Render everything every n iterations
        iter++;
        if (iter == 10000){
            SDL_SetRenderDrawColor(render, 10,10,10,255);
            SDL_RenderClear(render);
            for(int i = 0; i < nodes.size(); i++){
                if (nodesToBeRendered[i])
                    drawNode(i, false, from);
            }
            for (int i = 0; i < edges.size(); i++){
                if (edgesToBeRendered[i])
                    drawEdge(i, false);
            }
            SDL_RenderPresent(render);
            iter = 0;
        }
        
    }

    SDL_SetRenderDrawColor(render, 10,10,10,255);
    SDL_RenderClear(render);
    for(int i = 0; i < nodes.size(); i++){
        if (nodesToBeRendered[i])
            drawNode(i, false, from);
    }
    for (int i = 0; i < edges.size(); i++){
        if (edgesToBeRendered[i])
            drawEdge(i, false);
    }
    SDL_RenderPresent(render);

    f.close();
}

//Depth first search-based random maze generation
void DFS(){
    
    SDL_SetRenderDrawColor(render, 10,10,10,255);
    SDL_RenderClear(render);

    createEdgesAndVertices(nodes, edges);

    //Can be uncommented for non-random end points
    //int from = 0;   //Top left
    int from = rand()%(h*w);

    stack<int> currNode;
    priority_queue<int, vector<int>, CompareEdge> pq;
    vector<bool> visited(nodes.size(), false);

    currNode.push(from);
    visited[from] = true;
    while (!currNode.empty()){
        int u = currNode.top();
        currNode.pop();

        //Remove these lines for instant generation
        drawNode(u, false, from);
        SDL_RenderPresent(render);
        SDL_Delay(DFS_DELAY);
        
        vector<int> adj = getAdjacent(u);
        while(!adj.empty()){
            int v = adj.back();
            adj.pop_back();
            if (!visited[v])
                pq.push(getEdge(u, v));
        }

        SDL_SetRenderDrawColor(render, 75, 75, 75, 255);
        while (!pq.empty()){
            
            int v = pq.top();
            pq.pop();
            edges[v].masked = false;
            int adjNode = (u == edges[v].from ? edges[v].to : edges[v].from);
            visited[adjNode] = true;
            currNode.push(adjNode);

            //Remove these lines for instant generation
            drawEdge(v, false);
            SDL_RenderPresent(render);
            SDL_Delay(DFS_DELAY);
        }
    }

    //Uncomment this for instant generation

    /*
    //Draw background
    SDL_SetRenderDrawColor(render, 10,10,10,255);
    SDL_RenderClear(render);

    //Render nodes
    //The red and blue nodes are respectively the start and end point of the pathfinfing algorithm
    int index = 0;
    for (auto &node : nodes){
        SDL_Rect rect;
        rect.x = node.x;
        rect.y = node.y;
        rect.h = rect.w = RECT_SIZE;

        SDL_SetRenderDrawColor(render, 75, 75, 75, 255);
        if (index == from){
            SDL_SetRenderDrawColor(render, 255, 75, 75, 255);
        }

        SDL_RenderFillRect(render, &rect);

        index++;
    }

    //Render all unmasked edges
    SDL_SetRenderDrawColor(render, 75, 75, 75, 255);
    for (auto &edge : edges){
        if (edge.masked)
            continue;
        SDL_Rect rect;
        rect.x = edge.x;
        rect.y = edge.y;
        rect.w = rect.h = RECT_SIZE;
        SDL_RenderFillRect(render, &rect);
    }
    */
    SDL_RenderPresent(render);

}

//Dijkstra's pathfinding algorithm-based random maze generation
//Assign a random weight to each edge and let the algorithm find a minimum spanning tree
void Dijkstra(){

    createEdgesAndVertices(nodes, edges);

    //Can be uncommented for non-random end points
    //int from = 0;   //Top left
    //int to = h*w-1; //Bottom right
    int from = rand()%(h*w);
    int to = rand()%(h*w);

    while (from == to){
        to = rand()%(h*w);
    }

    //Array that stores the parent of each node in the minimum spanning tree.
    //It will be used to connect nodes to their parent nodes.
    vector<int> parents(nodes.size(), -1);

    priority_queue<NodeDistanceId> pq;
    vector<int> distances = vector<int>(nodes.size(), h*w*3000);
    distances[from] = 0;

    NodeDistanceId source(from, distances[from]);
    pq.push(source);
    while(!pq.empty()){
        NodeDistanceId u = pq.top();
        pq.pop();

        //Adjacent nodes to the unqueued node
        //Will be added only if it's not out of bounds
        priority_queue<NodeDistanceId> adj;
        //top
        if (u.id > w-1){
            adj.push(NodeDistanceId(u.id-w, u.distance + edges[getEdge(u.id-w, u.id)].weight));
        }
        //left
        if (u.id%w != 0){
            adj.push(NodeDistanceId(u.id-1, u.distance + edges[getEdge(u.id-1, u.id)].weight));
        }
        //right
        if (u.id%w != w-1){
            adj.push(NodeDistanceId(u.id+1, u.distance + edges[getEdge(u.id+1, u.id)].weight));
        }
        //bottom
        if (u.id < (h-1)*w){ 
            adj.push(NodeDistanceId(u.id+w, u.distance + edges[getEdge(u.id+w, u.id)].weight));
        }

        //Unqueue adjacent nodes and update distances
        while (!adj.empty()){
            NodeDistanceId v = adj.top();
            adj.pop();
            if (distances[v.id] > distances[u.id] + v.distance){
                distances[v.id] = distances[u.id] + v.distance;
                parents[v.id] = u.id;
                pq.push(v);
            }
        }
    }

    //Unmasks (makes visible) only edges that connect a parent node to a child node
    int index = 0;
    for (auto &parent : parents){
        int edge = getEdge(index, parent);
        if (edge != -1){
            edges[edge].masked = false;
        }
        index++;
    }

    //Draw background
    SDL_SetRenderDrawColor(render, 10,10,10,255);
    SDL_RenderClear(render);

    //Render nodes
    //The red and blue nodes are respectively the start and end point of the pathfinfing algorithm
    index = 0;
    for (auto &node : nodes){
        drawNode(index, false, from);
        index++;
    }

    //Render all unmasked edges
    SDL_SetRenderDrawColor(render, 75, 75, 75, 255);
    index = -1;
    for (auto &edge : edges){
        index++;
        if (edge.masked)
            continue;
        drawEdge(index, false);
    }

    SDL_RenderPresent(render);
}

bool innit(){
    if (SDL_Init(SDL_INIT_VIDEO) < 0){
        cout << "SDL couldn't be initialized: " << SDL_GetError() << "\n";
        return false;
    } 
    
    window = SDL_CreateWindow("Random Maze", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == NULL){
        cout << "Window couldn't be created: " << SDL_GetError() << "\n";
        return false;
    }

    return true;
}

void close(){
    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(render);
    screenSurface = NULL;
    window = NULL;
    SDL_Quit();
}

int main(int argc, char* args[] ){
    srand((unsigned int)time(0));
    rng = std::default_random_engine();

    if (!innit())   //SDL initialization
        return 0;

    render = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetRenderDrawColor(render, 10,10,10,255);
    SDL_RenderClear(render);

    SDL_Event e;
    bool quit = false;
    while(!quit){

        //Event catcher
        while(SDL_PollEvent(&e) != 0){
            if (e.type == SDL_QUIT){
                quit = true; 
            }
            else if (e.type == SDL_KEYDOWN){
                switch(e.key.keysym.sym){
                    case SDLK_ESCAPE:
                        quit = true;
                        break;
                    case SDLK_d:
                        Dijkstra();
                        break;
                    case SDLK_f:
                        DFS();
                        break;
                    case SDLK_g:
                        UniquePath();
                        break;
                }
            }
        }
    }
    
    close();
    
    return 0;
}