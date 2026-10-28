// MapParser.hpp
#pragma once
#include "MapData.hpp"
#include <unordered_map>
#include <vector>
#include <string>

class MapParser {
public:
    MapParser(const std::string& filepath); // Конструктор принимает путь к файлу .osm
    bool loadMap();
    // Геттеры для передачи данных в Navigator и Render
    const std::unordered_map<long long, Node>& getNodes() const { return nodes; }
    const std::unordered_map<long long, std::vector<Edge>>& getGraph() const { return graph; }
    // Границы карты
    double min_lat, max_lat, min_lon, max_lon;

private:
    std::string path;
    std::unordered_map<long long, Node> nodes;
    std::unordered_map<long long, std::vector<Edge>> graph;

    // Теги way, которые нужны для решения "можно ли по нему ходить пешком"
    struct WayTags {
        std::string highway; // тип дороги: footway, path, residential ...
        std::string foot;    // доступ для пешеходов: yes / no / designated ...
        std::string access;  // общий доступ: yes / no / private ...
        std::string area;    // area=yes -> way описывает площадь (полигон), а не линию
    };

    double calculateDistance(double lat1, double lon1, double lat2, double lon2);
    // type теперь передаётся снаружи (значение тега highway)
    void addMapEdge(long long from, long long to, const std::string& type);
    // Возвращает "" если way пригоден для пешехода, иначе - короткую причину отказа
    static std::string wayRejectReason(const WayTags& tags);
};