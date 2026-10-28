// App.cpp — точка входа нового клиента GraphNavigator.
// Экраны: 1) выбор карты (пакеты .gnpk из папки maps), 2) 3D-карта с панелью маршрута.
// Маршрут считается в RouteWorker (фоновый поток), поэтому окно не подвисает.
#include "data/Package.hpp"
#include "render/CityRenderer.hpp"
#include "routing/Router.hpp"
#include "ui/Widgets.hpp"
#include "raymath.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;
using namespace gn;
namespace th = gn::ui::theme;

namespace {

struct MapEntry { std::string path, title; uintmax_t bytes = 0; };

std::vector<MapEntry> findMaps(const std::string& extra) {
    std::vector<MapEntry> out;
    auto add = [&](const fs::path& p) {
        std::error_code ec;
        auto size = fs::file_size(p, ec);
        out.push_back({ p.string(), p.stem().string(), ec ? 0 : size });
    };
    if (!extra.empty() && fs::exists(extra)) add(extra);
    for (const char* dir : { "maps", "../maps", "../../maps" }) {
        std::error_code ec;
        if (!fs::is_directory(dir, ec)) continue;
        for (auto& e : fs::directory_iterator(dir, ec))
            if (e.path().extension() == ".gnpk") add(e.path());
        if (!out.empty()) break;
    }
    return out;
}

std::string formatDistance(double m) {
    return m < 1000 ? TextFormat("%d м", int(std::round(m))) : TextFormat("%.1f км", m / 1000);
}
std::string formatWalk(double m) {   // пешком 5 км/ч
    int min = std::max(1, int(std::round(m / 5000.0 * 60)));
    return min < 60 ? TextFormat("%d мин", min) : TextFormat("%d ч %d мин", min / 60, min % 60);
}
const char* kAlgNames[] = { "A*", "Dijkstra", "Жадный TSP" };

// Камера «как в картах»: точка фокуса на земле, азимут, наклон и расстояние.
struct OrbitCamera {
    Vector3 target{};
    float yaw = 0.6f, pitch = 0.9f, dist = 1500;
    Camera3D cam() const {
        Camera3D c{};
        c.target = target;
        c.position = { target.x + dist * std::cos(pitch) * std::sin(yaw), dist * std::sin(pitch),
                       target.z + dist * std::cos(pitch) * std::cos(yaw) };
        c.up = { 0, 1, 0 }; c.fovy = 45; c.projection = CAMERA_PERSPECTIVE;
        return c;
    }
};

// Пересечение луча из курсора с плоскостью земли y=0.
bool groundHit(Vector2 mouse, const Camera3D& c, Vector3& hit) {
    Ray r = GetScreenToWorldRay(mouse, c);
    if (std::fabs(r.direction.y) < 1e-5f) return false;
    float t = -r.position.y / r.direction.y;
    if (t <= 0) return false;
    hit = Vector3Add(r.position, Vector3Scale(r.direction, t));
    return true;
}

struct Toast { std::string text; double until = 0; Color color = th::text; };

class App {
public:
    explicit App(std::string extra) : m_maps(findMaps(extra)) {}
    void run();
private:
    void drawPicker();
    void openMap(const MapEntry& e);
    void frameMap();
    void handleCamera();
    void drawPanel();
    void drawControls();
    void drawStatus();
    void requestRoute();
    void pickPoint(uint32_t node);
    std::string nodeLabel(uint32_t node) const;
    void toast(const std::string& s, Color c = th::text) { m_toast = { s, GetTime() + 3.5, c }; }

    std::vector<MapEntry> m_maps;
    std::string m_error;
    std::unique_ptr<CityPackage> m_pkg;
    std::unique_ptr<RouteWorker> m_worker;
    std::unique_ptr<Router> m_router;
    CityRenderer m_render;
    OrbitCamera m_cam;
    std::string m_title;

    // Состояние панели маршрута
    std::string m_from, m_to;
    int m_focus = 0;                       // 0 нет, 1 «откуда», 2 «куда»
    uint32_t m_fromNode = UINT32_MAX, m_toNode = UINT32_MAX;
    std::vector<uint32_t> m_via;           // промежуточные точки (Shift+клик)
    int m_alg = 0;                         // 0..2 алгоритм, 3 сравнение
    std::vector<Route> m_results;
    bool m_resultsCompare = false;
    Toast m_toast;
    Vector2 m_pressPos{};
    bool m_dragging = false;
};

void App::run() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(1600, 900, "GraphNavigator — пешеходные маршруты");
    SetWindowMinSize(1100, 680);
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);                  // Esc не закрывает окно, а снимает фокус
    ui::loadFonts();
    if (m_maps.size() == 1) openMap(m_maps[0]);
    while (!WindowShouldClose()) {
        ui::beginFrame();
        BeginDrawing();
        if (!m_pkg) drawPicker(); else frameMap();
        ui::endFrame();
        EndDrawing();
    }
    m_worker.reset();
    m_render.unload();
    ui::unloadFonts();
    CloseWindow();
}

void App::openMap(const MapEntry& e) {
    auto pkg = std::make_unique<CityPackage>();
    std::string err;
    if (!loadPackage(e.path, *pkg, err)) { m_error = "Не удалось открыть «" + e.title + "»: " + err; return; }
    m_worker.reset();
    m_pkg = std::move(pkg);
    m_router = std::make_unique<Router>(m_pkg->graph);
    m_worker = std::make_unique<RouteWorker>(m_pkg->graph);
    m_render.build(*m_pkg);
    m_cam = {}; m_cam.target = m_render.center(); m_cam.dist = std::clamp(m_render.radius() * 1.6f, 300.f, 12000.f);
    m_title = e.title;
    m_from.clear(); m_to.clear(); m_via.clear(); m_results.clear();
    m_fromNode = m_toNode = UINT32_MAX; m_error.clear();
    toast("Карта загружена. Нажмите на карту, чтобы выбрать старт и финиш.");
}

void App::drawPicker() {
    ClearBackground(th::bg);
    float W = float(GetScreenWidth()), H = float(GetScreenHeight());
    // Шапка-баннер
    DrawRectangleGradientV(0, 0, int(W), int(H * 0.42f), Color{ 26, 115, 232, 255 }, Color{ 66, 133, 244, 255 });
    ui::text("GraphNavigator", 80, 70, 44, WHITE, true);
    ui::text("Пешеходные маршруты по городу в 3D", 82, 124, 22, Color{ 225, 236, 255, 255 });

    Rectangle c{ 80, H * 0.42f - 60, std::min(760.f, W - 160), std::max(260.f, H * 0.5f) };
    ui::card(c, 16);
    ui::text("Выберите карту", c.x + 28, c.y + 24, 24, th::text, true);
    float y = c.y + 74;
    if (m_maps.empty()) {
        ui::text("Карт пока нет. Соберите пакет командой:", c.x + 28, y, 18, th::muted);
        ui::text("python mapbake/mapbake.py район.osm maps/район.gnpk", c.x + 28, y + 30, 18, th::accent);
    }
    for (auto& m : m_maps) {
        Rectangle row{ c.x + 16, y, c.width - 32, 64 };
        bool h = ui::hovered(row);
        if (h) DrawRectangleRounded(row, 0.2f, 8, th::hover);
        DrawCircle(int(row.x + 34), int(row.y + 32), 18, th::accentSoft);
        ui::textCentered("M", { row.x + 16, row.y + 14, 36, 36 }, 18, th::accent, true);
        ui::text(m.title, row.x + 66, row.y + 10, 20, th::text, true);
        ui::text(TextFormat("Скачано · %.1f МБ", m.bytes / 1048576.0), row.x + 66, row.y + 36, 15, th::muted);
        if (ui::button({ row.x + row.width - 130, row.y + 14, 116, 36 }, "Открыть", true) ||
            (h && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))) openMap(m);
        y += 72;
        if (y > c.y + c.height - 70) break;
    }
    if (!m_error.empty()) ui::text(m_error, c.x + 28, c.y + c.height - 40, 16, th::red);
    ui::text("Данные карты © участники OpenStreetMap", 80, H - 36, 15, th::muted);
}

void App::frameMap() {
    if (std::vector<Route> res; m_worker->poll(res, m_resultsCompare)) {
        m_results = std::move(res);
        if (m_results.empty() || !m_results[0].success) toast("Маршрут между точками не найден", th::red);
    }
    handleCamera();
    ClearBackground(Color{ 196, 214, 176, 255 });
    Camera3D cam = m_cam.cam();
    BeginMode3D(cam);
    std::vector<uint32_t> markers;
    if (m_fromNode != UINT32_MAX) markers.push_back(m_fromNode);
    markers.insert(markers.end(), m_via.begin(), m_via.end());
    if (m_toNode != UINT32_MAX) markers.push_back(m_toNode);
    static const std::vector<uint32_t> none;
    m_render.draw(cam, m_results.empty() || !m_results[0].success ? none : m_results[0].nodes, markers, float(GetTime()));
    EndMode3D();

    // Подписи мест вблизи камеры (не больше 40, чтобы не захламлять экран).
    int shown = 0;
    for (auto& n : m_pkg->names) {
        if (shown >= 40) break;
        Vec2f p = m_pkg->graph.nodes[n.node];
        Vector3 w{ p.x, 2, p.z };
        if (Vector3Distance(w, cam.position) > m_cam.dist * 1.2f) continue;
        Vector2 s = GetWorldToScreen(w, cam);
        if (s.x < 0 || s.y < 0 || s.x > GetScreenWidth() || s.y > GetScreenHeight()) continue;
        float tw = ui::textWidth(n.name, 14, true);
        ui::text(n.name, s.x - tw / 2 + 1, s.y + 1, 14, Color{ 255, 255, 255, 200 }, true);
        ui::text(n.name, s.x - tw / 2, s.y, 14, Color{ 60, 64, 72, 255 }, true);
        ++shown;
    }
    drawPanel();
    drawControls();
    drawStatus();
    if (GetTime() < m_toast.until) {
        float W = float(GetScreenWidth());
        float tw = ui::textWidth(m_toast.text, 16) + 40;
        Rectangle r{ (W - tw) / 2, float(GetScreenHeight()) - 96, tw, 42 };
        DrawRectangleRounded(r, 0.5f, 8, Color{ 50, 52, 56, 235 });
        ui::textCentered(m_toast.text, r, 16, m_toast.color.r == th::text.r ? WHITE : Color{ 255, 170, 160, 255 });
    }
}

void App::handleCamera() {
    Vector2 m = GetMousePosition();
    bool overUi = ui::mouseOverUi();
    if (!overUi) {
        float wheel = GetMouseWheelMove();
        if (wheel != 0) m_cam.dist = std::clamp(m_cam.dist * std::pow(0.88f, wheel), 40.f, 20000.f);
    }
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !overUi) { m_pressPos = m; m_dragging = true; }
    if (m_dragging && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        // Перетаскивание: сдвигаем фокус так, чтобы точка под курсором «прилипла» к нему.
        Vector2 d = GetMouseDelta();
        float k = m_cam.dist / float(GetScreenHeight()) * 0.9f;
        float s = std::sin(m_cam.yaw), c = std::cos(m_cam.yaw);
        m_cam.target.x -= (d.x * c + d.y * s) * k;
        m_cam.target.z -= (-d.x * s + d.y * c) * k;
    }
    if (m_dragging && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        m_dragging = false;
        // Короткий клик без перемещения — выбор точки маршрута.
        if (Vector2Distance(m, m_pressPos) < 5) {
            Vector3 hit;
            if (groundHit(m, m_cam.cam(), hit)) {
                uint32_t n = m_router->nearestNode({ hit.x, hit.z });
                if (n != UINT32_MAX) pickPoint(n);
            }
        }
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !overUi) {
        Vector2 d = GetMouseDelta();
        m_cam.yaw -= d.x * 0.006f;
        m_cam.pitch = std::clamp(m_cam.pitch + d.y * 0.004f, 0.18f, 1.52f);
    }
    if (m_focus == 0) {   // клавиатура, когда не печатаем
        float step = m_cam.dist * 0.02f;
        float s = std::sin(m_cam.yaw), c = std::cos(m_cam.yaw);
        if (IsKeyDown(KEY_W)) { m_cam.target.x -= s * step; m_cam.target.z -= c * step; }
        if (IsKeyDown(KEY_S)) { m_cam.target.x += s * step; m_cam.target.z += c * step; }
        if (IsKeyDown(KEY_A)) { m_cam.target.x -= c * step; m_cam.target.z += s * step; }
        if (IsKeyDown(KEY_D)) { m_cam.target.x += c * step; m_cam.target.z -= s * step; }
        if (IsKeyDown(KEY_Q)) m_cam.yaw += 0.03f;
        if (IsKeyDown(KEY_E)) m_cam.yaw -= 0.03f;
    }
    if (IsKeyPressed(KEY_ESCAPE)) m_focus = 0;
}

std::string App::nodeLabel(uint32_t node) const {
    for (auto& n : m_pkg->names) if (n.node == node) return n.name;
    Vec2f p = m_pkg->graph.nodes[node];
    return TextFormat("Точка на карте (%.0f, %.0f)", p.x, -p.z);
}

void App::pickPoint(uint32_t n) {
    bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
    if (shift && m_fromNode != UINT32_MAX && m_toNode != UINT32_MAX) {
        m_via.push_back(n); toast("Добавлена промежуточная точка");
    } else if (m_focus == 2 || (m_fromNode != UINT32_MAX && m_focus != 1)) {
        m_toNode = n; m_to = nodeLabel(n);
    } else {
        m_fromNode = n; m_from = nodeLabel(n);
    }
    m_focus = 0;
    requestRoute();
}

void App::requestRoute() {
    if (m_fromNode == UINT32_MAX || m_toNode == UINT32_MAX) { m_results.clear(); return; }
    std::vector<uint32_t> pts{ m_fromNode };
    pts.insert(pts.end(), m_via.begin(), m_via.end());
    pts.push_back(m_toNode);
    if (m_alg == 3) m_worker->requestCompare(pts);
    else m_worker->request(Algorithm(m_alg), pts);
}

void App::drawPanel() {
    Rectangle p{ 20, 20, 420, 0 };
    bool hasResult = !m_results.empty();
    p.height = 262 + (hasResult ? (m_resultsCompare ? 170 : 128) : 0) + (m_via.empty() ? 0 : 30);
    ui::card(p, 14);
    float x = p.x + 20, y = p.y + 18;
    if (ui::iconButton({ x + 12, y + 12 }, 15, "\u2190")) {   // назад к выбору карт
        m_worker.reset(); m_router.reset(); m_render.unload(); m_pkg.reset(); return;
    }
    ui::text(m_title, x + 38, y + 1, 20, th::text, true);
    y += 44;

    Rectangle f1{ x, y, p.width - 90, 44 }, f2{ x, y + 52, p.width - 90, 44 };
    if (ui::textField(f1, m_from, "Откуда", m_focus == 1, th::green)) m_focus = 1;
    if (ui::textField(f2, m_to, "Куда", m_focus == 2, th::red)) m_focus = 2;
    if (ui::iconButton({ p.x + p.width - 42, y + 48 }, 18, "\u21C5")) {   // поменять местами
        std::swap(m_from, m_to); std::swap(m_fromNode, m_toNode);
        std::reverse(m_via.begin(), m_via.end());
        requestRoute();
    }
    y += 108;

    if (!m_via.empty()) {
        ui::text(TextFormat("Промежуточных точек: %d", int(m_via.size())), x, y + 4, 15, th::muted);
        if (ui::button({ p.x + p.width - 120, y, 100, 26 }, "Убрать")) { m_via.clear(); requestRoute(); }
        y += 30;
    }

    const char* seg[] = { "A*", "Dijkstra", "TSP", "Сравнить" };
    float sw = (p.width - 40) / 4;
    for (int i = 0; i < 4; ++i)
        if (ui::segment({ x + i * sw, y, sw - 4, 34 }, seg[i], m_alg == i)) { m_alg = i; requestRoute(); }
    y += 46;

    bool ready = m_fromNode != UINT32_MAX && m_toNode != UINT32_MAX;
    if (ui::button({ x, y, p.width - 40, 42 }, m_worker->busy() ? "Строим маршрут…" : "Построить маршрут", true, ready))
        requestRoute();
    y += 56;

    if (hasResult) {
        DrawLine(int(p.x), int(y), int(p.x + p.width), int(y), th::border);
        y += 14;
        const Route& r = m_results[0];
        if (!r.success) ui::text("Маршрут не найден: точки не связаны дорожками", x, y, 16, th::red);
        else {
            ui::text(formatWalk(r.lengthM), x, y, 30, th::text, true);
            ui::text(formatDistance(r.lengthM) + " · пешком", x + ui::textWidth(formatWalk(r.lengthM), 30, true) + 12, y + 10, 17, th::muted);
        }
        y += 44;
        if (!m_resultsCompare) {
            ui::text(TextFormat("%s: %.3f мс, раскрыто вершин: %zu", kAlgNames[m_alg], r.timeMs, r.visited), x, y, 15, th::muted);
            ui::text(TextFormat("Маршрут: %zu вершин графа", r.nodes.size()), x, y + 22, 15, th::muted);
        } else {
            ui::text("Алгоритм", x, y, 14, th::muted, true);
            ui::text("Время", x + 150, y, 14, th::muted, true);
            ui::text("Вершин", x + 240, y, 14, th::muted, true);
            ui::text("Длина", x + 310, y, 14, th::muted, true);
            double best = 1e300;
            for (auto& q : m_results) if (q.success) best = std::min(best, q.timeMs);
            for (size_t i = 0; i < m_results.size() && i < 3; ++i) {
                const Route& q = m_results[i];
                float ry = y + 26 + i * 28;
                if (q.success && q.timeMs == best) DrawRectangleRounded({ x - 8, ry - 4, p.width - 24, 26 }, 0.4f, 6, th::accentSoft);
                ui::text(kAlgNames[i], x, ry, 16, th::text, true);
                ui::text(q.success ? TextFormat("%.3f мс", q.timeMs) : "—", x + 150, ry, 16, th::text);
                ui::text(TextFormat("%zu", q.visited), x + 240, ry, 16, th::text);
                ui::text(q.success ? formatDistance(q.lengthM) : "нет пути", x + 310, ry, 16, th::text);
            }
        }
    }

    // Подсказки поиска под активным полем.
    if (m_focus) {
        std::string& q = m_focus == 1 ? m_from : m_to;
        std::string nq = ui::normalize(q);
        std::vector<const NamedPlace*> hits;
        if (!nq.empty())
            for (auto& n : m_pkg->names) {
                if (ui::normalize(n.name).find(nq) == std::string::npos) continue;
                bool dup = false;
                for (auto* h : hits) if (h->name == n.name) dup = true;
                if (!dup) hits.push_back(&n);
                if (hits.size() >= 6) break;
            }
        Rectangle field = m_focus == 1 ? f1 : f2;
        if (!hits.empty()) {
            Rectangle d{ field.x, field.y + field.height + 6, field.width, 40.f * hits.size() + 8 };
            ui::card(d, 10);
            for (size_t i = 0; i < hits.size(); ++i) {
                Rectangle row{ d.x + 4, d.y + 4 + 40.f * i, d.width - 8, 40 };
                bool h = ui::hovered(row);
                if (h) DrawRectangleRounded(row, 0.3f, 6, th::hover);
                ui::text(hits[i]->name, row.x + 14, row.y + 10, 17, th::text);
                if (h && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                    (m_focus == 1 ? m_fromNode : m_toNode) = hits[i]->node;
                    q = hits[i]->name; m_focus = 0;
                    // Камера летит к выбранному месту.
                    Vec2f pt = m_pkg->graph.nodes[hits[i]->node];
                    m_cam.target = { pt.x, 0, pt.z };
                    requestRoute();
                    return;
                }
            }
        } else if (!nq.empty()) {
            Rectangle d{ field.x, field.y + field.height + 6, field.width, 44 };
            ui::card(d, 10);
            ui::text("Ничего не найдено", d.x + 18, d.y + 12, 16, th::muted);
        }
        if (IsKeyPressed(KEY_ENTER) && !hits.empty()) {
            (m_focus == 1 ? m_fromNode : m_toNode) = hits[0]->node;
            q = hits[0]->name; m_focus = 0; requestRoute();
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::hovered(f1) && !ui::hovered(f2)) m_focus = 0;
    }
}

void App::drawControls() {
    float W = float(GetScreenWidth()), H = float(GetScreenHeight());
    float x = W - 44;
    if (ui::iconButton({ x, H - 210 }, 20, "+")) m_cam.dist = std::max(40.f, m_cam.dist * 0.75f);
    if (ui::iconButton({ x, H - 166 }, 20, "\u2212")) m_cam.dist = std::min(20000.f, m_cam.dist / 0.75f);
    // Компас: стрелка показывает север, клик — повернуть картой на север.
    Vector2 c{ x, H - 110 };
    if (ui::iconButton(c, 20, "")) { m_cam.yaw = 0; m_cam.pitch = 0.9f; }
    float a = m_cam.yaw;
    Vector2 tip{ c.x - std::sin(a) * 13, c.y - std::cos(a) * 13 };
    Vector2 l{ c.x - std::cos(a) * 5, c.y + std::sin(a) * 5 }, r{ c.x + std::cos(a) * 5, c.y - std::sin(a) * 5 };
    Vector2 tail{ c.x + std::sin(a) * 13, c.y + std::cos(a) * 13 };
    DrawTriangle(tip, l, r, th::red); DrawTriangle(tip, r, l, th::red);
    DrawTriangle(tail, r, l, th::muted); DrawTriangle(tail, l, r, th::muted);
    if (ui::button({ W - 150, 20, 130, 36 }, "Сбросить") ) {
        m_from.clear(); m_to.clear(); m_via.clear(); m_results.clear(); m_fromNode = m_toNode = UINT32_MAX;
    }
}

void App::drawStatus() {
    float W = float(GetScreenWidth()), H = float(GetScreenHeight());
    DrawRectangle(0, int(H - 30), int(W), 30, Color{ 255, 255, 255, 220 });
    auto& s = m_render.stats();
    ui::text(TextFormat("Зданий: %zu · Деревьев: %zu · Переходов: %zu · Вершин графа: %zu",
        m_pkg->buildings.size(), m_pkg->trees.size(), m_pkg->crossings.size(), m_pkg->graph.nodeCount()), 16, H - 24, 14, th::muted);
    std::string right = TextFormat("Кварталов в кадре: %d/%d · %d FPS · © OpenStreetMap", s.chunksDrawn, s.chunksTotal, GetFPS());
    ui::text(right, W - ui::textWidth(right, 14) - 16, H - 24, 14, th::muted);
    ui::text("ЛКМ — двигать / выбрать точку · ПКМ — вращать · колесо — масштаб · Shift+клик — через точку",
             W / 2 - 300, H - 24, 14, Color{ 150, 154, 160, 255 });
    ui::blockMouse({ 0, H - 30, W, 30 });
}

} // namespace

int main(int argc, char** argv) {
    App app(argc > 1 ? argv[1] : "");
    app.run();
    return 0;
}
