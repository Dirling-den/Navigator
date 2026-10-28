#pragma once

#include <vector>
#include <unordered_map>
#include <queue>
#include <chrono>
#include <algorithm>
#include "MapData.hpp" // Подключаем вашу обновленную структуру ShortestPath, Node, Edge

class Navigator {
public:
    // Конструктор принимает ссылки на данные из парсера
    explicit Navigator(const std::unordered_map<long long, Node>& nodes,
        const std::unordered_map<long long, std::vector<Edge>>& adjacencyList);

    // 1. Алгоритм A* (поиск между одной точкой старта и одной точкой финиша)
    ShortestPath findPathAStar(long long startId, long long goalId);

    // 2. Multi-source Dijkstra (поиск от нескольких стартов к одной цели)
    ShortestPath findPathMultiSource(const std::vector<long long>& startIds, long long goalId);

    // 3. Greedy TSP (Жадный коммивояжер для обхода списка точек)
    ShortestPath findPathGreedyTSP(const std::vector<long long>& visitIds);

private:
    const std::unordered_map<long long, Node>& m_nodes;
    const std::unordered_map<long long, std::vector<Edge>>& m_graph;

    // Кастомная эвристика (Расстояние Хаверсина между координатами)
    double calculateHeuristic(long long fromId, long long toId) const;

    // Вспомогательная функция для восстановления пути из хэш-таблицы предков
    std::vector<long long> reconstructPath(const std::unordered_map<long long, long long>& cameFrom, long long currentId) const;

    // Вспомогательная функция для подсчета реальной длины получившегося пути
    double calculatePathLength(const std::vector<long long>& path) const;
};
