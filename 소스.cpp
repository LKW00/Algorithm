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

// ��� ����ü ����
struct Node {
    int level;              // ���� Ž�� ����
    vector<int> path;       // ���� ���
    number bound;           // ���� ����� ��谪
};

// �켱���� ť�� ���� ����� ���� �� ������
struct CompareNode {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->bound > rhs->bound; // ��谪�� �� ū ��尡 ���� ó���ǵ��� ��
    }
};

// Branch and Bound �˰��򿡼� ����� ��谪�� ����ϴ� �Լ�
number calculateBound(Node u, const vector<vector<number>>& W) {
    number bound = 0;
    int n = W.size();
    vector<bool> visited(n, false); // �湮 ���θ� ����

    // ��λ��� ���� �湮 ó�� �� ��� ���� ���
    for (int i = 0; i < u.path.size() - 1; i++) {
        bound += W[u.path[i] - 1][u.path[i + 1] - 1]; // -1�� 0 ��� �ε����� ���� ��
        visited[u.path[i] - 1] = true;
    }
    visited[u.path.back() - 1] = true; // ������ ���� �湮 ó��

    // ���� ���õ��� �ּ� ���� ����� ��谪�� �߰�
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

    return bound; // ���� ��谪 ��ȯ
}

// Branch and Bound �˰��򿡼� ����� ���̸� ����ϴ� �Լ�
number calculateLength(Node u, const vector<vector<number>>& W) {
    number length = 0;
    for (int i = 0; i < u.path.size() - 1; i++) {
        length += W[u.path[i] - 1][u.path[i + 1] - 1]; // -1�� 0 ��� �ε����� ���� ��
    }
    return length; // ���� ��� ���� ��ȯ
}

// Branch and Bound �˰��򿡼� �ڽ� ��带 ó���ϴ� �Լ�
void takeCareOfChildren(Node v, int n, const vector<vector<number>>& W, priority_queue<Node*, vector<Node*>, CompareNode>& PQ, vector<int>& optTour, number& minLength) {
    Node u; // �ڽ� ���
    u.level = v.level + 1; // �ڽ� ����� ������ �θ� ����� ���� + 1
    for (int i = 2; i <= n; i++) { // ��� ������ ���� ���ø� Ž��
        if (find(v.path.begin(), v.path.end(), i) == v.path.end()) { // ���� �湮���� ���� ����
            u.path = v.path; // �θ� ����� ��θ� ����
            u.path.push_back(i); // ���� ���ø� ��ο� �߰�
            if (u.level == n - 2) { // ��� ���ø� �湮���� ��
                for (int j = 1; j <= n; j++) {
                    if (find(u.path.begin(), u.path.end(), j) == u.path.end()) {
                        u.path.push_back(j); // ���� ������ ���ø� ��ο� �߰�
                        break;
                    }
                }
                u.path.push_back(1); // ���� ���÷� ���ư�
                number length = calculateLength(u, W); // ��� ���� ���
                if (length < minLength) { // �ּ� ��� ���� ����
                    minLength = length;
                    optTour = u.path; // ���� ��� ����
                }
            }
            else { // ���� ��� ���ø� �湮���� �ʾ��� ��
                u.bound = calculateBound(u, W); // �ڽ� ����� ��谪 ���
                if (u.bound < minLength) { // �ڽ� ����� ��谪�� ���� �ּ� ��� ���̺��� ������
                    PQ.push(new Node(u)); // �ڽ� ��带 �켱���� ť�� �߰�
                }
            }
        }
    }
}

// TSP�� ���� Branch and Bound �˰���

void travel2(int n, const vector<vector<number>>& W, vector<int>& optTour, number& minLength) {
    // ��带 ������ �켱���� ť
    priority_queue<Node*, vector<Node*>, CompareNode> PQ;
    Node v;
    v.level = 0; // ���� ����
    v.path = { 1 }; // ù ��° ���ÿ��� ����
    minLength = INF; // �ּ� ���̸� ���Ѵ�� �ʱ�ȭ
    v.bound = calculateBound(v, W); // �ʱ� ��忡 ���� ��踦 ���
    PQ.push(new Node(v)); // �ʱ� ��带 �켱���� ť�� �߰�

    // �켱���� ť�� �� ������ ���
    while (!PQ.empty()) {
        v = *PQ.top(); // ���� ���� ��踦 ���� ��带 ������
        PQ.pop(); // �켱���� ť���� �ش� ��带 ����
        // ����� ��谡 ���� �ּ� ���̺��� ������ �ڽ��� Ž��
        if (v.bound < minLength) {
            takeCareOfChildren(v, n, W, PQ, optTour, minLength); // ���� ����� �ڽ��� ó���ϴ� �Լ� ȣ��
        }
    }
}

// TSP�� ���� ��Ʈ��ŷ �˰���

void tspBacktrack(int pos, int n, const vector<vector<number>>& W, vector<bool>& visited, vector<int>& path, number currCost, number& minLength, vector<int>& optTour) {
    if (path.size() == n) {
        currCost += W[path.back() - 1][0]; // ���������� ���ư�
        if (currCost < minLength) {
            minLength = currCost;
            optTour = path;
            optTour.push_back(1); // ����Ŭ�� �ϼ�
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
    path.push_back(1); // ù ��° ���ÿ��� ����
    tspBacktrack(0, n, W, visited, path, 0, minLength, optTour);
}

int main() {
    // ��� ��� ���� (5x5)
    vector<vector<number>> costMatrix1 = {
        {0, 10, 56, 99, 55},
        {10, 0, 42, 25, 74},
        {77, 35, 0, 30, 12},
        {20, 55, 30, 0, 46},
        {11, 56, 55,11, 0}
    };

    // ��� ��� ���� (10x10)
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

    vector<int> optTour; // ���� ���
    number minLength, minLength1, minLength2, minLength3; // �ּ� ���̵�

    // Branch and Bound �˰��� ���� (5x5)
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

    // Branch and Bound �˰��� ���� (10x10)
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

    // ��Ʈ��ŷ �˰��� ���� (5x5)
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

    // ��Ʈ��ŷ �˰��� ���� (10x10)
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