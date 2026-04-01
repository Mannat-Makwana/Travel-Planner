#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>
#include <cmath>
using namespace std;

const int N = 36;

vector<vector<pair<int,int>>> graph = {
    {{1,2},{6,3}},                // 0
    {{0,2},{2,3},{7,2}},          // 1
    {{1,3},{3,4},{8,3}},          // 2
    {{2,4},{4,2},{9,4}},          // 3
    {{3,2},{5,3},{10,2}},         // 4
    {{4,3},{11,3}},               // 5

    {{0,3},{7,2},{12,4}},         // 6
    {{1,2},{6,2},{8,3},{13,2}},   // 7
    {{2,3},{7,3},{9,2},{14,4}},   // 8
    {{3,4},{8,2},{10,3},{15,3}},  // 9
    {{4,2},{9,3},{11,2},{16,4}},  // 10
    {{5,3},{10,2},{17,3}},        // 11

    {{6,4},{13,3},{18,2}},        // 12
    {{7,2},{12,3},{14,2},{19,4}}, // 13
    {{8,4},{13,2},{15,3},{20,2}}, // 14
    {{9,3},{14,3},{16,2},{21,3}}, // 15
    {{10,4},{15,2},{17,3},{22,2}},// 16
    {{11,3},{16,3},{23,4}},       // 17

    {{12,2},{19,3},{24,3}},       // 18
    {{13,4},{18,3},{20,2},{25,2}},// 19
    {{14,2},{19,2},{21,3},{26,3}},// 20
    {{15,3},{20,3},{22,2},{27,4}},// 21
    {{16,2},{21,2},{23,3},{28,2}},// 22
    {{17,4},{22,3},{29,3}},       // 23

    {{18,3},{25,2},{30,4}},       // 24
    {{19,2},{24,2},{26,3},{31,2}},// 25
    {{20,3},{25,3},{27,2},{32,3}},// 26
    {{21,4},{26,2},{28,3},{33,2}},// 27
    {{22,2},{27,3},{29,2},{34,3}},// 28
    {{23,3},{28,2},{35,4}},       // 29

    {{24,4},{31,3}},              // 30
    {{25,2},{30,3},{32,2}},       // 31
    {{26,3},{31,2},{33,3}},       // 32
    {{27,2},{32,3},{34,2}},       // 33
    {{28,3},{33,2},{35,3}},       // 34
    {{29,4},{34,3}}               // 35
};


vector<int> dijkstras(int source,int goal , vector<vector<pair<int,int>>>& graph){
    int n = graph.size();
    vector<int> distance(n,INT_MAX);
    priority_queue<pair<int,int>,vector<pair<int,int>>,greater<pair<int,int>>> pq;
    distance[source] = 0 ;
    pq.push({0,source});
    while(!pq.empty()){
        pair<int,int> current = pq.top();
        pq.pop();
        int d = current.first;
        int u = current.second; 
        if(d>distance[u]) {continue;}
        if(u == goal){
            break;
        }

        for(int i=0;i<graph[u].size();i++){
            int v= graph[u][i].first;
            int w= graph[u][i].second;

            if(distance[u]+w < distance[v]){
                distance[v] = distance[u]+w;
                pq.push(make_pair(distance[v],v));
            }
        }
    }
    return distance; 
}


double heuristic(int u, int g){
    int x1 = u/6; 
    int y1 = u%6;
    int x2 = g/6;
    int y2 = g%6; 
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

vector<int> Astar(int source,int goal,vector<vector<pair<int,int>>>& graph){
    int n= graph.size();
    vector<int> distance(n,INT_MAX);
    priority_queue<pair<int,int>,vector<pair<int,int>>,greater<pair<int,int>>> pq ; 
    distance[source] = 0 ;
    pq.push(make_pair(heuristic(source,goal),source));
    while(!pq.empty()){
        pair<int,int> cur = pq.top();
        pq.pop();
        int u = cur.second;
        if(u==goal){
            break;
        }
        for(int i=0;i<graph[u].size();i++){
            int v = graph[u][i].first;
            int w = graph[u][i].second;
            if(distance[u]+w < distance[v]){
                distance[v]= distance[u]+w;
                int priority = distance[v]+ heuristic(v,goal);
                pq.push(make_pair(priority,v));
            }
        }
    }
    return distance;
}

vector<vector<int>> floydWarshall(vector<vector<pair<int,int>>>& graph) {
    int n = graph.size();

    vector<vector<int>> dist(n, vector<int>(n, INT_MAX));

    for (int i=0;i < n;i++) {
        dist[i][i] = 0;
        for (int j=0;j<graph[i].size();j++) {
            int v= graph[i][j].first;
            int w= graph[i][j].second;
            dist[i][v] = w;
        }
    }
    for (int k=0;k<n;k++) {
        for (int i=0;i<n;i++) {
            for (int j=0;j<n;j++) {

                if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX &&
                    dist[i][k] + dist[k][j] < dist[i][j]) {

                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }

    return dist;
}


int main(){
    int source, goal;
    cout<<"Enter the Source(1-35): ";
    cin>>source;
    cout<<"Enter the Goal: ";
    cin>>goal;

    auto start1 = chrono::high_resolution_clock::now();
    vector<int> dist_dijkstra = dijkstras(source, goal, graph);
    auto end1 = chrono::high_resolution_clock::now();
    auto time_dijkstra = chrono::duration_cast<chrono::microseconds>(end1 - start1);

    cout << "Dijkstra Distance to goal: " << dist_dijkstra[goal] << endl;
    cout << "Dijkstra Time: " << time_dijkstra.count() << " microseconds\n";
    cout<<"Time Complexity of Dijkstra's Algorithm is : O((E+V)logV)\n\n";

    auto start2 = chrono::high_resolution_clock::now();
    vector<int> dist_astar = Astar(source, goal, graph);
    auto end2 = chrono::high_resolution_clock::now();
    auto time_astar = chrono::duration_cast<chrono::microseconds>(end2 - start2);

    cout << "A* Distance to goal: " << dist_astar[goal] << endl;
    cout << "A* Time: " << time_astar.count() << " microseconds\n";
    cout<<"Time Complexity of A* is : O((E+V)logV)\n\n";

    auto start3 = chrono::high_resolution_clock::now();
    vector<vector<int>> dist_fw = floydWarshall(graph);
    auto end3 = chrono::high_resolution_clock::now();
    auto time_fw = chrono::duration_cast<chrono::microseconds>(end3 - start3);
    

    cout << "Floyd-Warshall Distance to goal: " << dist_fw[source][goal] << endl;
    cout << "Floyd-Warshall Time: " << time_fw.count() << " microseconds\n";
    cout<<"Time Complexity of Floyd Warshal's Algorithm is : O(n^3)\n";
    return 0;
}