#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>
#include <set>
#include <chrono>

using namespace std;
using namespace std::chrono;    

typedef int number;
const int INF = INT_MAX;
#define N 5
#define M 10

// 노드 구조체 정의
struct Node {
    int level;              // 현재 탐색 레벨
    vector<int> path;       // 현재 경로
    number bound;           // 현재 경로의 경계값
};

// 우선순위 큐를 위한 사용자 정의 비교 연산자
struct CompareNode {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->bound > rhs->bound; // 경계값이 더 큰 노드가 먼저 처리되도록 함
    }
};

// Branch and Bound 알고리즘에서 노드의 경계값을 계산하는 함수
number calculateBound(Node u, const vector<vector<number>>& W) {
    number bound = 0;
    int n = W.size();
    vector<bool> visited(n, false); // 방문 여부를 추적

    // 경로상의 도시 방문 처리 및 경로 길이 계산
    for (int i = 0; i < u.path.size() - 1; i++) {
        bound += W[u.path[i] - 1][u.path[i + 1] - 1]; // -1은 0 기반 인덱스를 위한 것
        visited[u.path[i] - 1] = true;
    }
    visited[u.path.back() - 1] = true; // 마지막 도시 방문 처리

    // 남은 도시들의 최소 간선 비용을 경계값에 추가
    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            number min_edge = INF;
            for (int j = 0; j < n; j++) {
                if (!visited[j] && W[i][j] < min_edge) {
                    min_edge = W[i][j];
                }
            }
            bound += min_edge;
        }
    }

    return bound; // 계산된 경계값 반환
}

// Branch and Bound 알고리즘에서 경로의 길이를 계산하는 함수
number calculateLength(Node u, const vector<vector<number>>& W) {
    number length = 0;
    for (int i = 0; i < u.path.size() - 1; i++) {
        length += W[u.path[i] - 1][u.path[i + 1] - 1]; // -1은 0 기반 인덱스를 위한 것
    }
    return length; // 계산된 경로 길이 반환
}

// Branch and Bound 알고리즘에서 자식 노드를 처리하는 함수
void takeCareOfChildren(Node v, int n, const vector<vector<number>>& W, priority_queue<Node*, vector<Node*>, CompareNode>& PQ, vector<int>& optTour, number& minLength) {
    Node u; // 자식 노드
    u.level = v.level + 1; // 자식 노드의 레벨은 부모 노드의 레벨 + 1
    for (int i = 2; i <= n; i++) { // 모든 가능한 다음 도시를 탐색
        if (find(v.path.begin(), v.path.end(), i) == v.path.end()) { // 아직 방문하지 않은 도시
            u.path = v.path; // 부모 노드의 경로를 복사
            u.path.push_back(i); // 다음 도시를 경로에 추가
            if (u.level == n - 2) { // 모든 도시를 방문했을 때
                for (int j = 1; j <= n; j++) {
                    if (find(u.path.begin(), u.path.end(), j) == u.path.end()) {
                        u.path.push_back(j); // 남은 마지막 도시를 경로에 추가
                        break;
                    }
                }
                u.path.push_back(1); // 시작 도시로 돌아감
                number length = calculateLength(u, W); // 경로 길이 계산
                if (length < minLength) { // 최소 경로 길이 갱신
                    minLength = length;
                    optTour = u.path; // 최적 경로 갱신
                }
            }
            else { // 아직 모든 도시를 방문하지 않았을 때
                u.bound = calculateBound(u, W); // 자식 노드의 경계값 계산
                if (u.bound < minLength) { // 자식 노드의 경계값이 현재 최소 경로 길이보다 작으면
                    PQ.push(new Node(u)); // 자식 노드를 우선순위 큐에 추가
                }
            }
        }
    }
}

// TSP를 위한 Branch and Bound 알고리즘

void travel2(int n, const vector<vector<number>>& W, vector<int>& optTour, number& minLength) {
    // 노드를 저장할 우선순위 큐
    priority_queue<Node*, vector<Node*>, CompareNode> PQ;
    Node v;
    v.level = 0; // 시작 레벨
    v.path = { 1 }; // 첫 번째 도시에서 시작
    minLength = INF; // 최소 길이를 무한대로 초기화
    v.bound = calculateBound(v, W); // 초기 노드에 대한 경계를 계산
    PQ.push(new Node(v)); // 초기 노드를 우선순위 큐에 추가

    // 우선순위 큐가 빌 때까지 계속
    while (!PQ.empty()) {
        v = *PQ.top(); // 가장 작은 경계를 가진 노드를 가져옴
        PQ.pop(); // 우선순위 큐에서 해당 노드를 제거
        // 노드의 경계가 현재 최소 길이보다 작으면 자식을 탐색
        if (v.bound < minLength) {
            takeCareOfChildren(v, n, W, PQ, optTour, minLength); // 현재 노드의 자식을 처리하는 함수 호출
        }
    }
}

// TSP를 위한 백트래킹 알고리즘

void tspBacktrack(int pos, int n, const vector<vector<number>>& W, vector<bool>& visited, vector<int>& path, number currCost, number& minLength, vector<int>& optTour) {
    if (path.size() == n) {
        currCost += W[path.back() - 1][0]; // 시작점으로 돌아감
        if (currCost < minLength) {
            minLength = currCost;
            optTour = path;
            optTour.push_back(1); // 사이클을 완성
        }
        return;
    }
    for (int i = 1; i < n; i++) {
        if (!visited[i]) {
            visited[i] = true;
            path.push_back(i + 1);
            tspBacktrack(i, n, W, visited, path, currCost + W[path[path.size() - 2] - 1][i], minLength, optTour);
            path.pop_back();
            visited[i] = false;
        }
    }
}

void travelBacktrack(int n, const vector<vector<number>>& W, vector<int>& optTour, number& minLength) {
    vector<bool> visited(n, false);
    vector<int> path;
    minLength = INF;
    visited[0] = true;
    path.push_back(1); // 첫 번째 도시에서 시작
    tspBacktrack(0, n, W, visited, path, 0, minLength, optTour);
}

int main() {
    // 비용 행렬 정의 (5x5)
    vector<vector<number>> costMatrix1 = {
        {0, 10, 56, 99, 55},
        {10, 0, 42, 25, 74},
        {77, 35, 0, 30, 12},
        {20, 55, 30, 0, 46},
        {11, 56, 55,11, 0}
    };

    // 비용 행렬 정의 (10x10)
    vector<vector<number>> costMatrix2 = {
         {0,  1,  2,  3,  4, 5, 6, 7, 8, 9},
         {11, 0, 56, 99, 55,23,33,29,33,31},
         {22, 10, 0, 99, 55,23,33,29,33,31},
         {33, 10, 20, 0, 55,23,33,29,33,31},
         {44, 10, 30,99, 0, 23,33,29,30,31},
         {55, 10, 40,99, 55,0 ,33,10,30,31},
         {66, 10, 50,99, 55,23, 0,20,33,31},
         {77, 10, 60,99, 55,23,33, 0,33,31},
         {88, 10, 56,99, 55,23,33,29, 0,31},
         {99, 10, 56, 99,55,23,33,29,33, 0},
    };

    vector<int> optTour; // 최적 경로
    number minLength, minLength1, minLength2, minLength3; // 최소 길이들

    // Branch and Bound 알고리즘 실행 (5x5)
    auto start0 = high_resolution_clock::now();
    travel2(N, costMatrix1, optTour, minLength);
    auto stop0 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop0 - start0);

    cout << "Branch and Bound Optimal 5X5 Path: ";
    for (int i : optTour) {
        cout << i << " ";
    }
    cout << "\nBranch and Bound Minimum Cost: " << minLength << endl;
    cout << "Branch and Bound Execution Time: " << duration.count() << " microseconds" << endl << endl;

    // Branch and Bound 알고리즘 실행 (10x10)
    auto start1 = high_resolution_clock::now();
    travel2(M, costMatrix2, optTour, minLength1);
    auto stop1 = high_resolution_clock::now();
    auto duration1 = duration_cast<microseconds>(stop1 - start1);

    cout << "Branch and Bound Optimal 10X10 Path: ";
    for (int i : optTour) {
        cout << i << " ";
    }
    cout << "\nBranch and Bound Minimum Cost: " << minLength1 << endl;
    cout << "Branch and Bound Execution Time: " << duration1.count() << " microseconds" << endl << endl;

    // 백트래킹 알고리즘 실행 (5x5)
    auto start2 = high_resolution_clock::now();
    travelBacktrack(N, costMatrix1, optTour, minLength2);
    auto stop2 = high_resolution_clock::now();
    auto duration2 = duration_cast<microseconds>(stop2 - start2);

    cout << "Backtracking Optimal 5X5 Path: ";
    for (int i : optTour) {
        cout << i << " ";
    }
    cout << "\nBacktracking Minimum Cost: " << minLength2 << endl;
    cout << "Backtracking Execution Time: " << duration2.count() << " microseconds" << endl << endl;

    // 백트래킹 알고리즘 실행 (10x10)
    auto start3 = high_resolution_clock::now();
    travelBacktrack(M, costMatrix2, optTour, minLength3);
    auto stop3 = high_resolution_clock::now();
    auto duration3 = duration_cast<microseconds>(stop3 - start3);

    cout << "Backtracking Optimal 10X10 Path: ";
    for (int i : optTour) {
        cout << i << " ";
    }
    cout << "\nBacktracking Minimum Cost: " << minLength3 << endl;
    cout << "Backtracking Execution Time: " << duration3.count() << " microseconds" << endl;

    return 0;
}