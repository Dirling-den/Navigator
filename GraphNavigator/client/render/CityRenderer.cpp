#include "CityRenderer.hpp"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>

namespace gn {
namespace {

// Накопитель треугольников без индексов (у raylib индексы 16-битные,
// а в квадрате города вершин бывает больше 65535).
struct MeshBuilder {
    std::vector<float> pos, nrm;
    std::vector<unsigned char> col;
    void tri(Vector3 a, Vector3 b, Vector3 c, Color k) {
        Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a)));
        if (n.y < 0) n = Vector3Negate(n);   // для крыш/дорог; для стен знак не важен — освещение «запечено»
        for (Vector3 v : { a, b, c }) {
            pos.insert(pos.end(), { v.x, v.y, v.z });
            nrm.insert(nrm.end(), { n.x, n.y, n.z });
            col.insert(col.end(), { k.r, k.g, k.b, k.a });
        }
    }
    void quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color k) { tri(a, b, c, k); tri(a, c, d, k); }
    bool empty() const { return pos.empty(); }
    // Копирует данные в Mesh (raylib освобождает их через UnloadModel) и грузит в GPU.
    Model toModel() {
        Mesh m{};
        m.vertexCount = int(pos.size() / 3);
        m.triangleCount = m.vertexCount / 3;
        m.vertices = (float*)MemAlloc(unsigned(pos.size() * sizeof(float)));
        m.normals = (float*)MemAlloc(unsigned(nrm.size() * sizeof(float)));
        m.colors = (unsigned char*)MemAlloc(unsigned(col.size()));
        std::memcpy(m.vertices, pos.data(), pos.size() * sizeof(float));
        std::memcpy(m.normals, nrm.data(), nrm.size() * sizeof(float));
        std::memcpy(m.colors, col.data(), col.size());
        UploadMesh(&m, false);
        return LoadModelFromMesh(m);
    }
};

Color shade(Color c, float k) {
    return Color{ (unsigned char)std::min(255.f, c.r * k), (unsigned char)std::min(255.f, c.g * k),
                  (unsigned char)std::min(255.f, c.b * k), c.a };
}

// Простая освещённость «от солнца» для стены с направлением (dx, dz).
float wallLight(float dx, float dz) {
    float len = std::hypot(dx, dz); if (len < 1e-6f) return 0.8f;
    float nx = dz / len, nz = -dx / len;                       // нормаль стены
    return 0.62f + 0.33f * std::fabs(nx * 0.6f + nz * 0.8f);
}

double area2(const std::vector<Vec2f>& p) {
    double s = 0;
    for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) s += double(p[j].x) * p[i].z - double(p[i].x) * p[j].z;
    return s;
}

// Триангуляция многоугольника «отрезанием ушей». Для вогнутых контуров зданий
// веер давал бы крыши, вылезающие за стены. Если контур самопересекается — веер.
std::vector<std::array<size_t, 3>> triangulate(const std::vector<Vec2f>& p) {
    std::vector<std::array<size_t, 3>> out;
    std::vector<size_t> v(p.size());
    for (size_t i = 0; i < v.size(); ++i) v[i] = i;
    if (area2(p) < 0) std::reverse(v.begin(), v.end());       // приводим к одному обходу
    auto cross = [&](size_t a, size_t b, size_t c) {
        return (double(p[b].x) - p[a].x) * (double(p[c].z) - p[a].z) - (double(p[b].z) - p[a].z) * (double(p[c].x) - p[a].x);
    };
    auto inside = [&](size_t a, size_t b, size_t c, size_t q) {
        return cross(a, b, q) > 0 && cross(b, c, q) > 0 && cross(c, a, q) > 0;
    };
    size_t guard = 0;
    while (v.size() > 3 && guard++ < p.size() * p.size()) {
        bool cut = false;
        for (size_t i = 0; i < v.size(); ++i) {
            size_t a = v[(i + v.size() - 1) % v.size()], b = v[i], c = v[(i + 1) % v.size()];
            if (cross(a, b, c) <= 0) continue;
            bool ear = true;
            for (size_t q : v) if (q != a && q != b && q != c && inside(a, b, c, q)) { ear = false; break; }
            if (!ear) continue;
            out.push_back({ a, b, c }); v.erase(v.begin() + i); cut = true; break;
        }
        if (!cut) break;
    }
    if (v.size() == 3) out.push_back({ v[0], v[1], v[2] });
    else for (size_t i = 1; i + 1 < v.size(); ++i) out.push_back({ v[0], v[i], v[i + 1] });
    return out;
}

Color roadColor(uint8_t kind) {
    switch (kind) {
    case 1: return Color{ 214, 196, 160, 255 };   // тропинка
    case 2: return Color{ 200, 200, 196, 255 };   // пешеходная зона
    case 3: return Color{ 230, 150, 120, 255 };   // лестница
    case 5: return Color{ 92, 96, 108, 255 };     // магистраль
    case 6: return Color{ 245, 245, 245, 255 };   // переход
    default: return Color{ 120, 124, 134, 255 };  // улица
    }
}
// Высота ленты: широкие дороги ниже, пешеходные выше, чтобы не мерцали на пересечениях.
float roadY(uint8_t kind) {
    switch (kind) { case 5: return 0.05f; case 4: return 0.08f; case 6: return 0.16f; default: return 0.12f; }
}

} // namespace

void CityRenderer::unload() {
    for (auto& c : m_chunks) { if (c.hasB) UnloadModel(c.buildings); if (c.hasR) UnloadModel(c.roads); }
    m_chunks.clear();
    if (m_treeReady) { UnloadModel(m_tree); UnloadModel(m_crown); m_treeReady = false; }
}

void CityRenderer::build(const CityPackage& pkg) {
    unload();
    m_pkg = &pkg;
    m_minX = m_minZ = 1e30f; m_maxX = m_maxZ = -1e30f;
    auto grow = [&](Vec2f p) { m_minX = std::min(m_minX, p.x); m_maxX = std::max(m_maxX, p.x); m_minZ = std::min(m_minZ, p.z); m_maxZ = std::max(m_maxZ, p.z); };
    for (auto& p : pkg.graph.nodes) grow(p);
    for (auto& b : pkg.buildings) for (auto& p : b.outline) grow(p);
    for (auto& r : pkg.roads) for (auto& p : r.points) grow(p);
    if (m_minX > m_maxX) { m_minX = m_minZ = -100; m_maxX = m_maxZ = 100; }
    m_center = { (m_minX + m_maxX) / 2, 0, (m_minZ + m_maxZ) / 2 };
    m_radius = std::max(100.f, std::hypot(m_maxX - m_minX, m_maxZ - m_minZ) / 2);

    // Ключ квадрата по координатам; std::map — чтобы порядок был детерминированным.
    std::map<std::pair<int, int>, std::pair<MeshBuilder, MeshBuilder>> cells;
    auto cell = [&](float x, float z) { return std::make_pair(int(std::floor(x / CHUNK)), int(std::floor(z / CHUNK))); };

    for (auto& b : pkg.buildings) {
        float cx = 0, cz = 0; for (auto& p : b.outline) { cx += p.x; cz += p.z; }
        cx /= b.outline.size(); cz /= b.outline.size();
        MeshBuilder& mb = cells[cell(cx, cz)].first;
        // Цвет зависит от высоты: низкие — тёплые, высотки — холодные.
        float t = std::clamp(b.height / 80.f, 0.f, 1.f);
        Color base{ (unsigned char)(226 - 60 * t), (unsigned char)(218 - 40 * t), (unsigned char)(204 - 10 * t), 255 };
        for (size_t i = 0; i < b.outline.size(); ++i) {
            Vec2f a = b.outline[i], c = b.outline[(i + 1) % b.outline.size()];
            mb.quad({ a.x, 0, a.z }, { c.x, 0, c.z }, { c.x, b.height, c.z }, { a.x, b.height, a.z },
                    shade(base, wallLight(c.x - a.x, c.z - a.z)));
        }
        for (auto& tr : triangulate(b.outline)) {
            Vec2f p0 = b.outline[tr[0]], p1 = b.outline[tr[1]], p2 = b.outline[tr[2]];
            mb.tri({ p0.x, b.height, p0.z }, { p1.x, b.height, p1.z }, { p2.x, b.height, p2.z }, shade(base, 1.05f));
        }
    }
    for (auto& r : pkg.roads) {
        float hw = r.width / 2, y = roadY(r.kind);
        Color k = roadColor(r.kind);
        for (size_t i = 0; i + 1 < r.points.size(); ++i) {
            Vec2f a = r.points[i], b = r.points[i + 1];
            float dx = b.x - a.x, dz = b.z - a.z, len = std::hypot(dx, dz);
            if (len < 0.01f) continue;
            float nx = -dz / len * hw, nz = dx / len * hw;
            MeshBuilder& mb = cells[cell((a.x + b.x) / 2, (a.z + b.z) / 2)].second;
            if (r.kind == 6) {
                // «Зебра»: белые полосы через каждые 1.2 м на тёмной подложке.
                mb.quad({ a.x + nx, y - 0.02f, a.z + nz }, { b.x + nx, y - 0.02f, b.z + nz }, { b.x - nx, y - 0.02f, b.z - nz }, { a.x - nx, y - 0.02f, a.z - nz }, Color{ 70, 72, 80, 255 });
                for (float s = 0.2f; s + 0.6f < len; s += 1.2f) {
                    float s0 = s / len, s1 = (s + 0.6f) / len;
                    Vector3 p0{ a.x + dx * s0, y, a.z + dz * s0 }, p1{ a.x + dx * s1, y, a.z + dz * s1 };
                    mb.quad({ p0.x + nx * 1.6f, y, p0.z + nz * 1.6f }, { p1.x + nx * 1.6f, y, p1.z + nz * 1.6f },
                            { p1.x - nx * 1.6f, y, p1.z - nz * 1.6f }, { p0.x - nx * 1.6f, y, p0.z - nz * 1.6f }, k);
                }
                continue;
            }
            mb.quad({ a.x + nx, y, a.z + nz }, { b.x + nx, y, b.z + nz }, { b.x - nx, y, b.z - nz }, { a.x - nx, y, a.z - nz }, k);
            // Квадратик в вершине сглаживает стык соседних отрезков.
            mb.quad({ b.x - hw, y, b.z - hw }, { b.x + hw, y, b.z - hw }, { b.x + hw, y, b.z + hw }, { b.x - hw, y, b.z + hw }, k);
        }
    }
    for (auto& [key, mbs] : cells) {
        Chunk c;
        c.c = { (key.first + 0.5f) * CHUNK, 20, (key.second + 0.5f) * CHUNK };
        c.r = CHUNK * 0.75f + 150;   // запас на высокие здания и выход контуров за квадрат
        if (!mbs.first.empty()) { c.buildings = mbs.first.toModel(); c.hasB = true; }
        if (!mbs.second.empty()) { c.roads = mbs.second.toModel(); c.hasR = true; }
        m_chunks.push_back(c);
    }
    m_tree = LoadModelFromMesh(GenMeshCylinder(0.25f, 3.0f, 6));
    m_tree.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = Color{ 110, 80, 55, 255 };
    m_crown = LoadModelFromMesh(GenMeshSphere(2.4f, 8, 8));
    m_crown.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = Color{ 70, 150, 80, 255 };
    m_treeReady = true;
}

void CityRenderer::draw(const Camera3D& cam, const std::vector<uint32_t>& route,
                        const std::vector<uint32_t>& markers, float time) {
    m_stats = {}; m_stats.chunksTotal = int(m_chunks.size());
    if (!m_pkg) return;
    Vector3 fwd = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
    float camH = std::max(10.f, cam.position.y);
    float far = std::clamp(camH * 6.f, 1500.f, 20000.f);
    // Грубое отсечение: квадрат рисуется, если он не позади камеры и не дальше far.
    auto visible = [&](Vector3 c, float r) {
        Vector3 d = Vector3Subtract(c, cam.position);
        float along = Vector3DotProduct(d, fwd);
        return along > -r && Vector3Length(d) < far + r;
    };

    float w = m_maxX - m_minX + 2000, h = m_maxZ - m_minZ + 2000;
    DrawPlane(m_center, Vector2{ w, h }, Color{ 196, 214, 176, 255 });
    rlDisableBackfaceCulling();   // обход контуров в OSM бывает любым
    for (auto& c : m_chunks) {
        if (!visible(c.c, c.r)) continue;
        ++m_stats.chunksDrawn;
        if (c.hasR) DrawModel(c.roads, {}, 1, WHITE);
        if (c.hasB) DrawModel(c.buildings, {}, 1, WHITE);
    }
    rlEnableBackfaceCulling();

    // Деревья — только вблизи (до 900 м), иначе их тысячи на кадр.
    float treeFar = std::min(far, 900.f);
    for (auto& t : m_pkg->trees) {
        Vector3 p{ t.x, 0, t.z };
        if (Vector3Distance(p, cam.position) > treeFar || !visible(p, 5)) continue;
        DrawModel(m_tree, { t.x, 1.5f, t.z }, 1, WHITE);
        DrawModel(m_crown, { t.x, 4.2f, t.z }, 1, WHITE);
        ++m_stats.treesDrawn;
    }

    // Маршрут — светящаяся лента над дорогами, толщина растёт с высотой камеры.
    const auto& nodes = m_pkg->graph.nodes;
    float rw = std::clamp(camH / 180.f, 1.6f, 14.f);
    for (size_t i = 0; i + 1 < route.size(); ++i) {
        Vec2f a = nodes[route[i]], b = nodes[route[i + 1]];
        float dx = b.x - a.x, dz = b.z - a.z, len = std::hypot(dx, dz);
        if (len < 0.01f) continue;
        float nx = -dz / len * rw, nz = dx / len * rw, y = 0.6f;
        Color k{ 26, 115, 232, 255 };
        DrawTriangle3D({ a.x + nx, y, a.z + nz }, { b.x + nx, y, b.z + nz }, { b.x - nx, y, b.z - nz }, k);
        DrawTriangle3D({ a.x + nx, y, a.z + nz }, { b.x - nx, y, b.z - nz }, { a.x - nx, y, a.z - nz }, k);
        DrawTriangle3D({ a.x + nx, y, a.z + nz }, { b.x - nx, y, b.z - nz }, { b.x + nx, y, b.z + nz }, k);
        DrawTriangle3D({ a.x + nx, y, a.z + nz }, { a.x - nx, y, a.z - nz }, { b.x - nx, y, b.z - nz }, k);
        DrawCylinder({ b.x, y - 0.05f, b.z }, rw, rw, 0.05f, 10, k);
    }
    // Метки: зелёная — старт, красная — финиш, синие — промежуточные. Лёгкое «дыхание».
    float pulse = 1.0f + 0.12f * std::sin(time * 4.f);
    float ms = std::clamp(camH / 60.f, 3.f, 40.f) * pulse;
    for (size_t i = 0; i < markers.size(); ++i) {
        if (markers[i] >= nodes.size()) continue;
        Vec2f p = nodes[markers[i]];
        Color k = (i == 0) ? Color{ 52, 168, 83, 255 } : (i + 1 == markers.size()) ? Color{ 234, 67, 53, 255 } : Color{ 66, 133, 244, 255 };
        DrawCylinderEx({ p.x, 0, p.z }, { p.x, ms * 2.2f, p.z }, ms * 0.12f, ms * 0.12f, 6, k);
        DrawSphere({ p.x, ms * 2.4f, p.z }, ms * 0.55f, k);
        DrawSphere({ p.x, ms * 2.4f, p.z }, ms * 0.25f, WHITE);
    }
}

} // namespace gn
