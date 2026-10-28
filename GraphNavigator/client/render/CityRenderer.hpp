// CityRenderer.hpp — 3D-отрисовка района: земля, дороги-ленты, переходы-«зебры»,
// выдавленные здания, деревья, маршрут.
// Геометрия режется на квадраты CHUNK×CHUNK метров; каждый квадрат — свой меш,
// невидимые камере квадраты не рисуются (frustum culling по ограничивающей сфере).
#pragma once
#include "../data/Package.hpp"
#include "raylib.h"
#include <vector>

namespace gn {

struct RenderStats { int chunksDrawn = 0, chunksTotal = 0, treesDrawn = 0; };

class CityRenderer {
public:
    static constexpr float CHUNK = 400.0f;

    void build(const CityPackage& pkg);   // вызывать после InitWindow
    void unload();
    ~CityRenderer() { unload(); }

    // route — вершины графа; markers — выбранные точки (старт первым, финиш последним).
    void draw(const Camera3D& cam, const std::vector<uint32_t>& route,
              const std::vector<uint32_t>& markers, float time);
    const RenderStats& stats() const { return m_stats; }
    Vector3 center() const { return m_center; }
    float radius() const { return m_radius; }

private:
    struct Chunk { Model buildings{}, roads{}; bool hasB = false, hasR = false; Vector3 c{}; float r = 0; };
    const CityPackage* m_pkg = nullptr;
    std::vector<Chunk> m_chunks;
    Model m_tree{};
    Model m_crown{};
    bool m_treeReady = false;
    Vector3 m_center{};
    float m_radius = 100;
    float m_minX = 0, m_minZ = 0, m_maxX = 0, m_maxZ = 0;
    RenderStats m_stats;
};

} // namespace gn
