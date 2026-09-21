// ============================================================================
// DYNAMIC SPACE DEBRIS COLLISION DETECTION AND RISK-AWARE PATH PLANNING SYSTEM
// ----------------------------------------------------------------------------
// Compile:  g++ -std=c++17 -O2 -Wall -Wextra collision_avoidance_system.cpp -o cas
// Run:      ./cas
//
// NOTE: This is an educational SIMULATION. Motion is straight-line
// (position + velocity * time). Risk thresholds are configurable simulation
// values, NOT real aerospace safety standards.
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <vector>

using namespace std;

const double INF = numeric_limits<double>::infinity();
const double EPS = 1e-9;

// ----------------------------------------------------------------------------
// 3D vector
// ----------------------------------------------------------------------------
struct Vec3 {
    double x = 0, y = 0, z = 0;
};
Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const Vec3& a) { return sqrt(dot(a, a)); }
double distance3D(const Vec3& a, const Vec3& b) { return norm(a - b); }  // 3D Euclidean distance

// ----------------------------------------------------------------------------
// Risk levels + configurable thresholds
// "separation" = closest distance between satellite and debris SURFACE
//              = (center distance) - (debris radius), never below 0.
// ----------------------------------------------------------------------------
enum class RiskLevel { NONE = 0, LOW, MEDIUM, HIGH, CRITICAL };

string riskName(RiskLevel r) {
    switch (r) {
        case RiskLevel::LOW:      return "LOW";
        case RiskLevel::MEDIUM:   return "MEDIUM";
        case RiskLevel::HIGH:     return "HIGH";
        case RiskLevel::CRITICAL: return "CRITICAL";
        default:                  return "NONE";
    }
}

struct RiskConfig {
    double satellite_speed = 10.0;   // simulation units per second

    // Separation thresholds (simulation units). Separation above low_limit = safe.
    double low_limit      = 30.0;    // safety threshold: below this we start caring
    double medium_limit   = 20.0;
    double high_limit     = 12.0;
    double critical_limit = 5.0;

    // Penalties added to route cost (same units as fuel/distance).
    double low_penalty    = 5.0;
    double medium_penalty = 25.0;
    double high_penalty   = 100.0;
    // CRITICAL has no penalty: the edge is BLOCKED.

    RiskLevel classify(double separation) const {
        if (separation <= critical_limit) return RiskLevel::CRITICAL;
        if (separation <= high_limit)     return RiskLevel::HIGH;
        if (separation <= medium_limit)   return RiskLevel::MEDIUM;
        if (separation <= low_limit)      return RiskLevel::LOW;
        return RiskLevel::NONE;
    }
    double penalty(RiskLevel r) const {
        switch (r) {
            case RiskLevel::LOW:    return low_penalty;
            case RiskLevel::MEDIUM: return medium_penalty;
            case RiskLevel::HIGH:   return high_penalty;
            default:                return 0.0;
        }
    }
};

// ----------------------------------------------------------------------------
// Debris + trajectory prediction:  future_position = position + velocity * time
// ----------------------------------------------------------------------------
struct Debris {
    int id;
    Vec3 pos;      // position at time t = 0
    Vec3 vel;      // constant velocity
    double radius;

    Vec3 predict(double t) const { return pos + vel * t; }
};

// ----------------------------------------------------------------------------
// CollisionEvent + min-heap ordered by smallest time-to-collision
// (time_to_collision = time of closest approach, measured from mission start)
// ----------------------------------------------------------------------------
struct CollisionEvent {
    int debris_id;
    double time_to_collision;
    double min_separation;
    RiskLevel risk;
    int seg_from;   // extra info for printing: which route segment (node ids)
    int seg_to;
};

struct EventLater {   // "a is lower priority than b" -> gives a MIN-heap on time
    bool operator()(const CollisionEvent& a, const CollisionEvent& b) const {
        return a.time_to_collision > b.time_to_collision;
    }
};
using EventQueue = priority_queue<CollisionEvent, vector<CollisionEvent>, EventLater>;

// ----------------------------------------------------------------------------
// Collision detection for ONE straight segment (from -> to) vs ONE debris.
// Satellite moves at constant speed starting at t_start. Debris moves linearly.
// Relative position is linear in time, so the closest approach can be found
// with a small formula (minimise |a + b*tau| for tau in [0, duration]).
// Returns true (and fills ev) if the predicted separation is below the
// safety threshold (i.e. risk level != NONE).
// ----------------------------------------------------------------------------
bool analyzeSegment(const Vec3& from, const Vec3& to, double t_start, const Debris& d,
                    const RiskConfig& cfg, CollisionEvent& ev, int from_id = -1, int to_id = -1) {
    Vec3 seg = to - from;
    double len = norm(seg);
    double duration = len / cfg.satellite_speed;
    Vec3 sat_vel = (len > EPS) ? seg * (cfg.satellite_speed / len) : Vec3{};

    Vec3 a = from - d.predict(t_start);   // relative position at start of segment
    Vec3 b = sat_vel - d.vel;             // relative velocity
    double bb = dot(b, b);

    double tau = 0.0;                     // time (after t_start) of closest approach
    if (bb > 1e-12) tau = -dot(a, b) / bb;
    tau = max(0.0, min(tau, duration));   // stay inside this segment's time window

    double center_dist = norm(a + b * tau);
    double separation = max(0.0, center_dist - d.radius);

    RiskLevel level = cfg.classify(separation);
    if (level == RiskLevel::NONE) return false;
    ev = {d.id, t_start + tau, separation, level, from_id, to_id};
    return true;
}

// ----------------------------------------------------------------------------
// Orbital graph: nodes with 3D coordinates + adjacency list of weighted edges
// ----------------------------------------------------------------------------
struct Node {
    int id;
    string name;
    Vec3 pos;
    bool hazard;   // static hazard zone: never entered by any A* variant
};
struct Edge {
    int to;
    double weight;   // fuel cost = Euclidean distance
};

class Graph {
public:
    vector<Node> nodes;
    vector<vector<Edge>> adj;

    int addNode(const string& name, Vec3 pos, bool hazard = false) {
        int id = (int)nodes.size();
        nodes.push_back({id, name, pos, hazard});
        adj.emplace_back();
        return id;
    }
    void addEdge(int u, int v) {   // bidirectional
        double w = distance3D(nodes[u].pos, nodes[v].pos);
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
    }
};

// ----------------------------------------------------------------------------
// Result of a route search / evaluation
// ----------------------------------------------------------------------------
struct PathResult {
    bool found = false;
    vector<int> path;
    double fuel = 0;            // sum of edge distances
    double risk_penalty = 0;    // sum of penalties (LOW/MEDIUM/HIGH encounters)
    double total = 0;           // fuel + risk_penalty
    int critical_count = 0;     // CRITICAL encounters (only nonzero for basic A*)
    int expanded = 0;           // nodes popped and processed
    int stale_skipped = 0;      // outdated priority_queue entries ignored
};

// ----------------------------------------------------------------------------
// A* search (same algorithm for both modes).
//   riskAware = false : classic A*, cost = fuel only, debris ignored
//   riskAware = true  : cost = fuel + risk penalty, CRITICAL edges are blocked
// g_score = total cost so far (fuel + penalty). fuel_score is tracked
// separately because arrival TIME at a node = fuel_so_far / speed, and the
// debris position depends on that time.
// Heuristic = 3D Euclidean distance (never overestimates, since penalties >= 0).
// Simplification: each node keeps only its best-known cost/time state.
// ----------------------------------------------------------------------------
double heuristic(const Graph& g, int a, int b) { return distance3D(g.nodes[a].pos, g.nodes[b].pos); }

PathResult aStar(const Graph& g, int start, int goal, const vector<Debris>& debris,
                 const RiskConfig& cfg, bool riskAware) {
    int n = (int)g.nodes.size();
    vector<double> g_score(n, INF), f_score(n, INF), fuel_score(n, INF), pen_score(n, INF);
    vector<int> came_from(n, -1);

    struct QEntry {
        double f;
        double g;   // g at push time -> lets us detect stale entries
        int node;
    };
    auto cmp = [](const QEntry& a, const QEntry& b) { return a.f > b.f; };   // min-heap on f
    priority_queue<QEntry, vector<QEntry>, decltype(cmp)> open(cmp);

    PathResult res;
    if (g.nodes[start].hazard || g.nodes[goal].hazard) return res;

    g_score[start] = 0; fuel_score[start] = 0; pen_score[start] = 0;
    f_score[start] = heuristic(g, start, goal);
    open.push({f_score[start], 0.0, start});

    while (!open.empty()) {
        QEntry cur = open.top();
        open.pop();

        // Stale/duplicate entry: a cheaper route to this node was found after
        // this entry was pushed. std::priority_queue can't update entries, so we
        // simply skip outdated ones.
        if (cur.g > g_score[cur.node] + EPS) { res.stale_skipped++; continue; }
        res.expanded++;

        int u = cur.node;
        if (u == goal) break;

        double t_u = fuel_score[u] / cfg.satellite_speed;   // arrival time at u

        for (const Edge& e : g.adj[u]) {
            int v = e.to;
            if (g.nodes[v].hazard) continue;

            double edge_pen = 0.0;
            if (riskAware) {
                bool blocked = false;
                for (const Debris& d : debris) {
                    CollisionEvent ev;
                    if (analyzeSegment(g.nodes[u].pos, g.nodes[v].pos, t_u, d, cfg, ev)) {
                        if (ev.risk == RiskLevel::CRITICAL) { blocked = true; break; }
                        edge_pen += cfg.penalty(ev.risk);
                    }
                }
                if (blocked) continue;   // CRITICAL = blocked edge
            }

            double tentative = g_score[u] + e.weight + edge_pen;
            if (tentative + EPS < g_score[v]) {
                came_from[v] = u;
                g_score[v] = tentative;
                fuel_score[v] = fuel_score[u] + e.weight;
                pen_score[v] = pen_score[u] + edge_pen;
                f_score[v] = tentative + heuristic(g, v, goal);
                open.push({f_score[v], tentative, v});   // old entries for v become stale
            }
        }
    }

    if (g_score[goal] == INF) return res;   // no path

    for (int v = goal; v != -1; v = came_from[v]) res.path.push_back(v);
    reverse(res.path.begin(), res.path.end());
    res.found = true;
    res.fuel = fuel_score[goal];
    res.risk_penalty = pen_score[goal];
    res.total = g_score[goal];
    return res;
}

// ----------------------------------------------------------------------------
// Walk along a route and collect every collision risk into a min-heap.
// Also fills fuel / penalty / critical_count of `r`.
// ----------------------------------------------------------------------------
void evaluateRoute(const Graph& g, PathResult& r, const vector<Debris>& debris,
                   const RiskConfig& cfg, EventQueue& events) {
    r.fuel = 0; r.risk_penalty = 0; r.critical_count = 0;
    for (size_t i = 0; i + 1 < r.path.size(); i++) {
        int u = r.path[i], v = r.path[i + 1];
        double t_start = r.fuel / cfg.satellite_speed;
        for (const Debris& d : debris) {
            CollisionEvent ev;
            if (analyzeSegment(g.nodes[u].pos, g.nodes[v].pos, t_start, d, cfg, ev, u, v)) {
                events.push(ev);
                if (ev.risk == RiskLevel::CRITICAL) r.critical_count++;
                else r.risk_penalty += cfg.penalty(ev.risk);
            }
        }
        r.fuel += distance3D(g.nodes[u].pos, g.nodes[v].pos);
    }
    r.total = r.fuel + r.risk_penalty;
}

// ----------------------------------------------------------------------------
// Scenarios + graph
// ----------------------------------------------------------------------------
struct Scenario {
    string name;
    string description;
    vector<Debris> debris;
    int start;
    int goal;
};

Graph buildGraph() {
    Graph g;
    int S = g.addNode("START", {0, 0, 0});          // 0
    int A = g.addNode("A",     {30, 20, 0});        // 1
    int B = g.addNode("B",     {30, -25, 0});       // 2
    int C = g.addNode("C",     {60, 0, 10});        // 3
    int D = g.addNode("D",     {60, 40, 0});        // 4
    int E = g.addNode("E(hazard)", {60, -40, 0}, true);   // 5  static hazard zone
    int G = g.addNode("GOAL",  {100, 0, 0});        // 6
    g.addEdge(S, A); g.addEdge(S, B);
    g.addEdge(A, C); g.addEdge(B, C);
    g.addEdge(A, D); g.addEdge(B, E);
    g.addEdge(C, G); g.addEdge(D, G); g.addEdge(E, G);
    return g;
}

vector<Scenario> buildScenarios() {
    vector<Scenario> s;

    s.push_back({"Normal route",
                 "No debris. Plain shortest path START -> GOAL.",
                 {}, 0, 6});

    s.push_back({"Route blocked by critical debris",
                 "Debris #1 crosses edge A-C when the satellite gets there.",
                 {{1, {45, 21, 5}, {0, -2, 0}, 2.0}}, 0, 6});

    s.push_back({"Safer alternative route",
                 "Debris #1 passes near edge C-GOAL (not a direct hit). The detour costs less than the penalty.",
                 {{1, {80, -13, 5}, {0, 0, 0}, 2.0}}, 0, 6});

    s.push_back({"Multiple debris",
                 "Three debris objects: one blocks A-C, one passes at moderate distance, one is far away.",
                 {{1, {45, 21, 5}, {0, -2, 0}, 2.0},
                  {2, {45, -17, 5}, {0, 0, 0}, 1.5},
                  {3, {150, 150, 150}, {-1, -1, -1}, 3.0}}, 0, 6});

    s.push_back({"No collision",
                 "Debris exist but stay far from every route segment.",
                 {{1, {50, 90, 40}, {1, 0, 0}, 2.0},
                  {2, {-40, -60, 10}, {0.5, 0.5, 0}, 1.0}}, 0, 6});

    s.push_back({"No safe path",
                 "Critical debris cover BOTH edges leaving START.",
                 {{1, {15, 7.3, 0}, {0, 1.5, 0}, 2.0},
                  {2, {15, -10.55, 0}, {0, -1, 0}, 2.0}}, 0, 6});
    return s;
}

// ----------------------------------------------------------------------------
// Printing helpers
// ----------------------------------------------------------------------------
string pathToString(const Graph& g, const vector<int>& p) {
    string s;
    for (size_t i = 0; i < p.size(); i++) {
        if (i) s += " -> ";
        s += g.nodes[p[i]].name;
    }
    return s;
}

void printThresholds(const RiskConfig& c) {
    cout << "\n--- Simulation risk thresholds (configurable, NOT real standards) ---\n"
         << fixed << setprecision(1)
         << "  Satellite speed : " << c.satellite_speed << " units/s\n"
         << "  CRITICAL        : separation <= " << c.critical_limit << "  -> edge BLOCKED\n"
         << "  HIGH            : separation <= " << c.high_limit << "  -> penalty " << c.high_penalty << "\n"
         << "  MEDIUM          : separation <= " << c.medium_limit << "  -> penalty " << c.medium_penalty << "\n"
         << "  LOW             : separation <= " << c.low_limit << "  -> penalty " << c.low_penalty << "\n"
         << "  Above " << c.low_limit << " = no risk\n";
}

void printGraph(const Graph& g) {
    cout << "\n--- Orbital graph ---\n" << fixed << setprecision(1);
    for (const Node& n : g.nodes) {
        cout << "  [" << n.id << "] " << left << setw(10) << n.name << right
             << " (" << n.pos.x << ", " << n.pos.y << ", " << n.pos.z << ")  ->";
        for (const Edge& e : g.adj[n.id]) cout << " " << e.to << "(" << e.weight << ")";
        cout << "\n";
    }
}

void printRouteSummary(const Graph& g, const string& title, const PathResult& r) {
    cout << "  " << title << "\n";
    if (!r.found) { cout << "    NO PATH FOUND\n"; return; }
    cout << "    Route        : " << pathToString(g, r.path) << "\n"
         << fixed << setprecision(2)
         << "    Fuel cost    : " << r.fuel << "\n"
         << "    Risk penalty : " << r.risk_penalty << "\n"
         << "    Total cost   : " << r.total << "\n";
    if (r.critical_count > 0)
        cout << "    WARNING      : route has " << r.critical_count
             << " CRITICAL encounter(s) -> UNSAFE\n";
    cout << "    (A* expanded " << r.expanded << " nodes, skipped " << r.stale_skipped
         << " stale queue entries)\n";
}

void printEvents(EventQueue events) {   // copy: popping must not destroy caller's heap
    if (events.empty()) { cout << "    None - no predicted separation below the safety threshold.\n"; return; }
    cout << "    (ordered by time, most imminent first)\n" << fixed << setprecision(2);
    bool first = true;
    while (!events.empty()) {
        CollisionEvent e = events.top();
        events.pop();
        cout << "    " << (first ? ">> " : "   ")
             << "Debris #" << e.debris_id << "  t=" << e.time_to_collision << "s"
             << "  min separation=" << e.min_separation
             << "  risk=" << riskName(e.risk)
             << "  (segment " << e.seg_from << "->" << e.seg_to << ")\n";
        first = false;
    }
}

void runScenario(const Graph& g, const Scenario& sc, const RiskConfig& cfg) {
    cout << "\n==============================================================\n"
         << " SCENARIO: " << sc.name << "\n"
         << " " << sc.description << "\n"
         << " Mission : " << g.nodes[sc.start].name << " -> " << g.nodes[sc.goal].name << "\n"
         << "==============================================================\n";

    cout << "Debris objects: " << sc.debris.size() << "\n" << fixed << setprecision(1);
    for (const Debris& d : sc.debris)
        cout << "  #" << d.id << " pos(" << d.pos.x << "," << d.pos.y << "," << d.pos.z
             << ") vel(" << d.vel.x << "," << d.vel.y << "," << d.vel.z << ") r=" << d.radius << "\n";

    // 1) Basic A*: fuel only. Then check what debris risk that route would face.
    PathResult basic = aStar(g, sc.start, sc.goal, sc.debris, cfg, false);
    EventQueue basicEvents;
    if (basic.found) evaluateRoute(g, basic, sc.debris, cfg, basicEvents);

    cout << "\n[1] Detected collision risks on the fuel-only (basic A*) route:\n";
    if (!basic.found) cout << "    (no path exists in the graph)\n";
    else printEvents(basicEvents);

    if (!basicEvents.empty()) {
        const CollisionEvent& m = basicEvents.top();
        cout << fixed << setprecision(2)
             << "\n[2] MOST IMMINENT COLLISION: Debris #" << m.debris_id
             << " at t=" << m.time_to_collision << "s, min separation "
             << m.min_separation << ", risk " << riskName(m.risk) << "\n";
    } else {
        cout << "\n[2] Most imminent collision: none\n";
    }

    // 2) Risk-aware A*
    PathResult safe = aStar(g, sc.start, sc.goal, sc.debris, cfg, true);
    EventQueue safeEvents;
    if (safe.found) evaluateRoute(g, safe, sc.debris, cfg, safeEvents);

    cout << "\n[3] Route comparison:\n";
    printRouteSummary(g, "Basic A* (fuel only, ignores debris):", basic);
    printRouteSummary(g, "Risk-aware A* (fuel + risk penalty, CRITICAL blocked):", safe);

    if (safe.found) {
        cout << "\n[4] Remaining risks on the SELECTED (risk-aware) route:\n";
        printEvents(safeEvents);
    } else {
        cout << "\n[4] No safe route exists: every path is blocked by CRITICAL debris\n"
             << "    or a hazard node. Recommended action: delay the maneuver / re-plan.\n";
    }
}

// ----------------------------------------------------------------------------
// Benchmark (real measurements, printed at runtime)
// ----------------------------------------------------------------------------
void runBenchmark(const Graph& g, const vector<Scenario>& scenarios, const RiskConfig& cfg) {
    const int RUNS = 20000;
    cout << "\n--- Benchmark: " << RUNS << " runs per scenario per algorithm ---\n"
         << "(measured on this machine with std::chrono; results vary by machine)\n"
         << left << setw(36) << "Scenario" << right << setw(14) << "Basic (us)"
         << setw(16) << "Risk-aware (us)" << "\n";

    volatile double sink = 0;   // stops the compiler from deleting the work
    for (const Scenario& sc : scenarios) {
        using clk = chrono::steady_clock;

        auto t0 = clk::now();
        for (int i = 0; i < RUNS; i++) sink = sink + aStar(g, sc.start, sc.goal, sc.debris, cfg, false).total;
        auto t1 = clk::now();
        for (int i = 0; i < RUNS; i++) sink = sink + aStar(g, sc.start, sc.goal, sc.debris, cfg, true).total;
        auto t2 = clk::now();

        double basicUs = chrono::duration<double, micro>(t1 - t0).count() / RUNS;
        double safeUs  = chrono::duration<double, micro>(t2 - t1).count() / RUNS;
        cout << left << setw(36) << sc.name << right << fixed << setprecision(3)
             << setw(14) << basicUs << setw(16) << safeUs << "\n";
    }
    cout << "Risk-aware A* does extra work per edge (collision checks against every debris).\n";
}

// ----------------------------------------------------------------------------
// Interactive CLI
// ----------------------------------------------------------------------------
bool readDouble(const string& prompt, double& out) {
    cout << prompt;
    if (cin >> out) return true;
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Invalid number.\n";
    return false;
}

void customScenario(const Graph& g, const RiskConfig& cfg) {
    Scenario sc;
    sc.name = "Custom scenario";
    sc.description = "User-defined debris.";
    double s, t, count;
    printGraph(g);
    if (!readDouble("Start node id: ", s) || !readDouble("Goal node id: ", t)) return;
    int N = (int)g.nodes.size();
    if (s < 0 || t < 0 || s >= N || t >= N) { cout << "Node id out of range.\n"; return; }
    sc.start = (int)s; sc.goal = (int)t;
    if (!readDouble("Number of debris objects (0-10): ", count)) return;
    if (count < 0 || count > 10) { cout << "Out of range.\n"; return; }

    for (int i = 0; i < (int)count; i++) {
        Debris d; d.id = i + 1;
        cout << "Debris #" << d.id << ":\n";
        if (!readDouble("  pos x: ", d.pos.x) || !readDouble("  pos y: ", d.pos.y) ||
            !readDouble("  pos z: ", d.pos.z) || !readDouble("  vel x: ", d.vel.x) ||
            !readDouble("  vel y: ", d.vel.y) || !readDouble("  vel z: ", d.vel.z) ||
            !readDouble("  radius: ", d.radius))
            return;
        sc.debris.push_back(d);
    }
    runScenario(g, sc, cfg);
}

int main() {
    Graph graph = buildGraph();
    RiskConfig cfg;
    vector<Scenario> scenarios = buildScenarios();

    cout << "=====================================================================\n"
         << " DYNAMIC SPACE DEBRIS COLLISION DETECTION AND RISK-AWARE PATH PLANNING\n"
         << " (educational simulation - straight-line motion, not real orbital data)\n"
         << "=====================================================================\n";

    while (true) {
        cout << "\n----- MENU -----\n"
             << " 1. Show orbital graph\n"
             << " 2. Show risk thresholds\n"
             << " 3. Run one built-in scenario\n"
             << " 4. Run ALL built-in scenarios\n"
             << " 5. Custom scenario (enter your own debris)\n"
             << " 6. Benchmark (basic A* vs risk-aware A*)\n"
             << " 0. Exit\n"
             << "Choice: ";
        int choice;
        if (!(cin >> choice)) break;

        if (choice == 0) break;
        else if (choice == 1) printGraph(graph);
        else if (choice == 2) printThresholds(cfg);
        else if (choice == 3) {
            for (size_t i = 0; i < scenarios.size(); i++)
                cout << "  " << i + 1 << ". " << scenarios[i].name << "\n";
            cout << "Scenario number: ";
            int k;
            if (cin >> k && k >= 1 && k <= (int)scenarios.size()) runScenario(graph, scenarios[k - 1], cfg);
            else { cin.clear(); cin.ignore(numeric_limits<streamsize>::max(), '\n'); cout << "Invalid.\n"; }
        }
        else if (choice == 4) for (const Scenario& sc : scenarios) runScenario(graph, sc, cfg);
        else if (choice == 5) customScenario(graph, cfg);
        else if (choice == 6) runBenchmark(graph, scenarios, cfg);
        else cout << "Invalid choice.\n";
    }
    cout << "Goodbye.\n";
    return 0;
}