#include <vector>

using std::vector;

struct Edge{
    int weight;
    int from;
    int to;
    int x, y;
    bool masked = true;
    
};


struct Node{
    int x, y;
    vector<int> edges;
} ;

struct NodeDistanceId{
    NodeDistanceId(int id, int distance){
        this->id = id;
        this->distance = distance;
    }
    int id;
    int distance = 255*3000;

    bool operator<(const NodeDistanceId &a) const{
        return distance > a.distance;
    }

} ;
