// Package.hpp — пакет города (.gnpk), созданный mapbake/mapbake.py.
// Формат описан в шапке mapbake.py. Каждое чтение проверяет границы буфера,
// поэтому повреждённый файл даёт ошибку, а не падение приложения.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace gn {

struct Vec2f { float x = 0, z = 0; };            // метры от центра города (север = -Z)

struct CsrEdge { uint32_t to; float weight; uint8_t type; };

// Граф в формате CSR: рёбра вершины i лежат в edges[offsets[i] .. offsets[i+1]).
struct CsrGraph {
    std::vector<Vec2f> nodes;
    std::vector<uint32_t> offsets;
    std::vector<CsrEdge> edges;
    size_t nodeCount() const { return nodes.size(); }
};

struct Building { float height = 10; std::vector<Vec2f> outline; };

// kind: 1 тропинка, 2 пешеходная зона, 3 лестница, 4 улица, 5 магистраль, 6 переход
struct Road { uint8_t kind = 1; float width = 2; std::vector<Vec2f> points; };

struct NamedPlace { uint32_t node; std::string name; };

struct CityPackage {
    double originLat = 0, originLon = 0;
    CsrGraph graph;
    std::vector<Building> buildings;
    std::vector<Vec2f> trees;
    std::vector<Road> roads;
    std::vector<uint32_t> crossings;
    std::vector<NamedPlace> names;
};

// Разбор из памяти. При ошибке возвращает false и пишет причину в error.
bool parsePackage(const std::vector<uint8_t>& bytes, CityPackage& out, std::string& error);
bool loadPackage(const std::string& path, CityPackage& out, std::string& error);

} // namespace gn
