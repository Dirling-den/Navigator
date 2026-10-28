#include "Navigator.hpp"
#include <cmath>
#include <limits>
#include <functional>

Navigator::Navigator(const std::unordered_map<long long, Node>& nodes,
    const std::unordered_map<long long, std::vector<Edge>>& adjacencyList)
    : m_nodes(nodes), m_graph(adjacencyList) {
}

double Navigator::calculateHeuristic(long long fromId, long long toId) const {
    auto fromIt = m_nodes.find(fromId);
    auto toIt = m_nodes.find(toId);

    if (fromIt == m_nodes.end() || toIt == m_nodes.end()) {
        return 0.0;
    }

    const double EarthRadiusMeters = 6371000.0;
    const double PI = 3.14159265358979323846;

    double lat1 = fromIt->second.lat * PI / 180.0;
    double lon1 = fromIt->second.lon * PI / 180.0;
    double lat2 = toIt->second.lat * PI / 180.0;
    double lon2 = toIt->second.lon * PI / 180.0;

    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;

    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
        std::cos(lat1) * std::cos(lat2) *
        std::sin(dLon / 2.0) * std::sin(dLon / 2.0);

    a = std::clamp(a, 0.0, 1.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return EarthRadiusMeters * c;
}

std::vector<long long> Navigator::reconstructPath(const std::unordered_map<long long, long long>& cameFrom, long long currentId) const {
    std::vector<long long> totalPath;
    totalPath.push_back(currentId);

    auto it = cameFrom.find(currentId);
    while (it != cameFrom.end()) {
        currentId = it->second;
        totalPath.push_back(currentId);
        it = cameFrom.find(currentId);
    }

    std::reverse(totalPath.begin(), totalPath.end());
    return totalPath;
}

double Navigator::calculatePathLength(const std::vector<long long>& path) const {
    if (path.size() < 2) return 0.0;

    double totalLength = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        long long u = path[i];
        long long v = path[i + 1];

        auto it = m_graph.find(u);
        if (it != m_graph.end()) {
            for (const auto& edge : it->second) {
                if (edge.to == v) {
                    totalLength += edge.weight; // Используем ваше поле weight
                    break;
                }
            }
        }
    }
    return totalLength;
}

// 1. Алгоритм A*
ShortestPath Navigator::findPathAStar(long long startId, long long goalId) {
    ShortestPath result;
    if (!m_nodes.count(startId) || !m_nodes.count(goalId)) return result;
    auto startTime = std::chrono::high_resolution_clock::now();

    std::unordered_map<long long, double> gScore;
    std::unordered_map<long long, double> fScore;

    struct QueueElement {
        long long id;
        double fScore;
        bool operator>(const QueueElement& other) const {
            return fScore > other.fScore;
        }
    };

    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> openSet;
    std::unordered_map<long long, long long> cameFrom;

    gScore[startId] = 0.0;
    double startFScore = calculateHeuristic(startId, goalId);
    fScore[startId] = startFScore;

    openSet.push({ startId, startFScore });

    while (!openSet.empty()) {
        QueueElement current = openSet.top();
        openSet.pop();
        long long currentId = current.id;

        if (fScore.find(currentId) != fScore.end() && current.fScore > fScore[currentId]) {
            continue;
        }

        if (currentId == goalId) {
            result.success = true;
            result.node_ids = reconstructPath(cameFrom, goalId); // Переименовано под node_ids
            result.total_length = gScore.at(goalId);
            break;
        }

        auto it = m_graph.find(currentId);
        if (it != m_graph.end()) {
            for (const auto& edge : it->second) {
                long long neighborId = edge.to;
                double tentativeGScore = gScore[currentId] + edge.weight; // Используем weight

                double neighborOldGScore = std::numeric_limits<double>::infinity();
                auto neighborGIt = gScore.find(neighborId);
                if (neighborGIt != gScore.end()) {
                    neighborOldGScore = neighborGIt->second;
                }

                if (tentativeGScore < neighborOldGScore) {
                    cameFrom[neighborId] = currentId;
                    gScore[neighborId] = tentativeGScore;
                    double neighborFScore = tentativeGScore + calculateHeuristic(neighborId, goalId);
                    fScore[neighborId] = neighborFScore;
                    openSet.push({ neighborId, neighborFScore });
                }
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.execution_time_ms = std::chrono::duration<double, std::milli>(endTime - startTime).count(); // Переименовано под execution_time_ms

    return result;
}

// 2. Multi-source Dijkstra
ShortestPath Navigator::findPathMultiSource(const std::vector<long long>& startIds, long long goalId) {
    ShortestPath result;
    if (startIds.empty() || !m_nodes.count(goalId)) return result;

    auto startTime = std::chrono::high_resolution_clock::now();

    std::unordered_map<long long, double> gScore;
    std::unordered_map<long long, long long> cameFrom;

    struct QueueElement {
        long long id;
        double distance;
        bool operator>(const QueueElement& other) const {
            return distance > other.distance;
        }
    };

    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> openSet;

    for (long long startId : startIds) {
        if (!m_nodes.count(startId)) continue;
        gScore[startId] = 0.0;
        openSet.push({ startId, 0.0 });
    }

    while (!openSet.empty()) {
        QueueElement current = openSet.top();
        openSet.pop();
        long long currentId = current.id;

        if (gScore.find(currentId) != gScore.end() && current.distance > gScore[currentId]) {
            continue;
        }

        if (currentId == goalId) {
            result.success = true;
            result.node_ids = reconstructPath(cameFrom, goalId);
            result.total_length = gScore.at(goalId);
            break;
        }

        auto it = m_graph.find(currentId);
        if (it != m_graph.end()) {
            for (const auto& edge : it->second) {
                long long neighborId = edge.to;
                double tentativeGScore = gScore[currentId] + edge.weight;

                double neighborOldGScore = std::numeric_limits<double>::infinity();
                auto neighborGIt = gScore.find(neighborId);
                if (neighborGIt != gScore.end()) {
                    neighborOldGScore = neighborGIt->second;
                }

                if (tentativeGScore < neighborOldGScore) {
                    cameFrom[neighborId] = currentId;
                    gScore[neighborId] = tentativeGScore;
                    openSet.push({ neighborId, tentativeGScore });
                }
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.execution_time_ms = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    return result;
}

// 3. Greedy TSP
ShortestPath Navigator::findPathGreedyTSP(const std::vector<long long>& visitIds) {
    ShortestPath result;
    for (auto id : visitIds) if (!m_nodes.count(id)) return result;
    if (visitIds.size() < 2) {
        if (!visitIds.empty()) {
            result.node_ids = visitIds;
            result.success = true;
        }
        return result;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<long long> unvisited = visitIds;
    long long currentId = unvisited.front();
    unvisited.erase(unvisited.begin());

    result.node_ids.push_back(currentId);
    result.success = true;

    while (!unvisited.empty()) {
        size_t nextIndex = 0;
        double minDistance = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < unvisited.size(); ++i) {
            double dist = calculateHeuristic(currentId, unvisited[i]);
            if (dist < minDistance) {
                minDistance = dist;
                nextIndex = i;
            }
        }

        long long nextTargetId = unvisited[nextIndex];
        unvisited.erase(unvisited.begin() + nextIndex);

        ShortestPath legResult = findPathAStar(currentId, nextTargetId);

        if (legResult.success && !legResult.node_ids.empty()) {
            result.node_ids.insert(result.node_ids.end(), legResult.node_ids.begin() + 1, legResult.node_ids.end());
            result.total_length += legResult.total_length;
        }
        else {
            result.success = false;
            break;
        }

        currentId = nextTargetId;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.execution_time_ms = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    return result;
}
