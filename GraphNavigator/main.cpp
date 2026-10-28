#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <algorithm>
#include <limits>
#include <string>
#include <cmath>
#include <cctype>
#include "raylib.h"
#include "MapData.hpp"
#include "MapParser.hpp"
#include "Navigator.hpp"

inline Font g_hudFont;
enum class AlgorithmType { A_STAR = 0, DIJKSTRA = 1, GREEDY_TSP = 2, BENCHMARK_COMPARE = 3 };
enum class AppState { INITIAL_CHOOSE = 0, ACTIVE_MAP = 1 };
enum class ActiveInput { NONE, START_INPUT, END_INPUT };

struct EventLog { std::string text; Color color; };
struct AlgResult { double timeMs; double distance; bool success; };

struct UIState {
    AppState currentState = AppState::INITIAL_CHOOSE;
    AlgorithmType activeAlgorithm = AlgorithmType::A_STAR;
    std::vector<long long> clickedNodeIds;
    ShortestPath currentRoute;
    bool isPathFound = false; bool needsCalculation = false;
    Vector2 menuPos = { 40.0f, 40.0f }; Vector2 menuSize = { 480.0f, 640.0f };
    bool isDragging = false; Vector2 dragOffset = { 0.0f, 0.0f };
    Vector2 cameraOffset = { 0.0f, 0.0f }; float cameraZoom = 1.0f;
    std::string startInputStr = ""; std::string endInputStr = "";
    ActiveInput focusedField = ActiveInput::NONE;
    double executionTime = 0.0; double routeLength = 0.0;
    std::unordered_map<int, AlgResult> benchResults;
    std::vector<EventLog> internalLogs;
    void LogEvent(const std::string& msg, Color clr = WHITE) {
        if (internalLogs.size() >= 4) internalLogs.erase(internalLogs.begin());
        internalLogs.push_back({ msg, clr });
    }
};

// Расстояние по формуле гаверсинусов (метры). Нужно для честных весов демо-графа,
// иначе эвристика A* (тоже в метрах) становится больше реальных весов и A* теряет оптимальность.
double GeoDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dLat = (lat2 - lat1) * PI / 180.0; double dLon = (lon2 - lon1) * PI / 180.0;
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) + std::cos(lat1 * PI / 180.0) * std::cos(lat2 * PI / 180.0) * std::sin(dLon / 2) * std::sin(dLon / 2);
    a = std::clamp(a, 0.0, 1.0);
    return R * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

void ScaleGraphTo3DSpace(std::unordered_map<long long, Node>& nodes, double min_lat, double max_lat, double min_lon, double max_lon, float mapWidth, float mapHeight) {
    // PI объявлен библиотекой raylib.
    // Градус долготы короче градуса широты в cos(lat) раз: без поправки карта растягивается по X
    double lonFactor = std::cos((min_lat + max_lat) / 2.0 * PI / 180.0);
    double geoWidth = (max_lon - min_lon) * lonFactor; double geoHeight = max_lat - min_lat;
    if (geoWidth <= 0.0 && geoHeight <= 0.0) {
        for (auto& [id, node] : nodes) { node.x = 0; node.y = 0; }
        return;
    }
    // Единый масштаб для обеих осей (сохраняем пропорции), карта центрируется в (0,0)
    double scale = std::min(mapWidth / geoWidth, mapHeight / geoHeight);
    double midLat = (min_lat + max_lat) / 2.0; double midLon = (min_lon + max_lon) / 2.0;
    for (auto& [id, node] : nodes) {
        node.x = static_cast<float>((node.lon - midLon) * lonFactor * scale);
        // Камера смотрит в сторону -Z, поэтому север (большая широта) должен иметь меньший Z,
        // иначе карта отображается зеркально по вертикали
        node.y = static_cast<float>(-(node.lat - midLat) * scale);
    }
}

// Поиск без учёта регистра для ASCII и русской кириллицы.
static std::string NormalizeName(const std::string& s) {
    std::string out;
    for (size_t i = 0; i < s.size();) {
        int bytes = 0;
        int codepoint = GetCodepointNext(s.c_str() + i, &bytes);
        i += static_cast<size_t>(std::max(bytes, 1));
        if (codepoint < 128 && std::isspace(static_cast<unsigned char>(codepoint))) continue;
        if (codepoint >= 'A' && codepoint <= 'Z') codepoint += 'a' - 'A';
        if (codepoint >= 0x410 && codepoint <= 0x42F) codepoint += 0x20;
        if (codepoint == 0x401) codepoint = 0x451;
        int length = 0;
        const char* encoded = CodepointToUTF8(codepoint, &length);
        out.append(encoded, length);
    }
    return out;
}

long long FindValidRouterNode(const std::unordered_map<long long, Node>& nodes, const std::unordered_map<long long, std::vector<Edge>>& graph, const std::string& text) {
    std::string target = NormalizeName(text);
    if (target.empty()) return -1;
    long long matchedId = -1;
    // Сначала точное совпадение, затем частичное (иначе "Вход" мог совпасть с "Вход 2" случайно)
    for (int pass = 0; pass < 2 && matchedId == -1; ++pass) {
        for (const auto& [id, node] : nodes) {
            if (node.name.empty()) continue;
            std::string candidate = NormalizeName(node.name);
            if (candidate.empty()) continue;
            bool ok = (pass == 0) ? (candidate == target)
                : (candidate.find(target) != std::string::npos);
            if (ok) { matchedId = id; break; }
        }
    }
    if (matchedId == -1) return -1;
    auto gIt = graph.find(matchedId);
    if (gIt != graph.end() && !gIt->second.empty()) return matchedId;
    long long nearestId = -1; float minDist = std::numeric_limits<float>::max();
    const Node& originNode = nodes.at(matchedId);
    for (const auto& [id, edges] : graph) {
        if (edges.empty()) continue;
        auto targetIt = nodes.find(id); if (targetIt == nodes.end()) continue;
        float dx = targetIt->second.x - originNode.x; float dy = targetIt->second.y - originNode.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < minDist) { minDist = dist; nearestId = id; }
    }
    return nearestId;
}

// Ввод текста с поддержкой UTF-8 (кириллица). Раньше принимались только коды 32..125,
// а Backspace удалял один байт и ломал многобайтовые символы.
void CaptureStringInput(std::string& str) {
    int key = GetCharPressed();
    while (key > 0) {
        if (key >= 32 && key != 127) {
            int len = 0; const char* utf8 = CodepointToUTF8(key, &len);
            str.append(utf8, len);
        }
        key = GetCharPressed();
    }
    if (IsKeyPressed(KEY_BACKSPACE) && !str.empty()) {
        // Удаляем байты-продолжения (10xxxxxx) и затем ведущий байт символа
        while (!str.empty() && (static_cast<unsigned char>(str.back()) & 0xC0) == 0x80) str.pop_back();
        if (!str.empty()) str.pop_back();
    }
}
void DrawControlHUD(UIState& ui, size_t totalNodes, bool isDemo) {
    Vector2 m = GetMousePosition();
    Color bgSoft = { 30, 32, 40, 245 }; Color headerSoft = { 42, 45, 55, 255 };
    Color textMain = { 255, 255, 255, 255 }; Color amberSoft = { 254, 215, 60, 255 }; Color inputBg = { 48, 51, 62, 255 };
    Rectangle mRect = { ui.menuPos.x, ui.menuPos.y, ui.menuSize.x, ui.menuSize.y };
    Rectangle hRect = { ui.menuPos.x, ui.menuPos.y, ui.menuSize.x, 55.0f };
    DrawRectangleRounded(mRect, 0.02f, 4, bgSoft); DrawRectangleRounded(hRect, 0.02f, 4, headerSoft);
    DrawRectangleRoundedLines(mRect, 0.02f, 4, Color{ 75, 80, 95, 255 });

    DrawText("GraphNavigator 3D Engine", (int)ui.menuPos.x + 20, (int)ui.menuPos.y + 16, 18, textMain);
    std::string alg = (ui.activeAlgorithm == AlgorithmType::BENCHMARK_COMPARE) ? "Mode: Real-time Multi-AI Benchmark" : "Core AI: Route Calculation Mode";
    DrawText(alg.c_str(), (int)ui.menuPos.x + 20, (int)ui.menuPos.y + 75, 14, amberSoft);
    DrawText(TextFormat("%s | Nodes: %i", isDemo ? "DEMO MAP" : "OSM MAP", static_cast<int>(totalNodes)),
        (int)ui.menuPos.x + 20, (int)ui.menuPos.y + 95, 12, GRAY);

    Rectangle inS = { ui.menuPos.x + 20, ui.menuPos.y + 115, ui.menuSize.x - 40, 48 };
    Rectangle inE = { ui.menuPos.x + 20, ui.menuPos.y + 180, ui.menuSize.x - 40, 48 };
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (CheckCollisionPointRec(m, inS)) ui.focusedField = ActiveInput::START_INPUT;
        else if (CheckCollisionPointRec(m, inE)) ui.focusedField = ActiveInput::END_INPUT;
        else if (!CheckCollisionPointRec(m, mRect)) ui.focusedField = ActiveInput::NONE;
    }
    DrawRectangleRounded(inS, 0.1f, 4, inputBg); DrawRectangleRounded(inE, 0.1f, 4, inputBg);
    if (ui.focusedField == ActiveInput::START_INPUT) DrawRectangleRoundedLines(inS, 0.1f, 4, amberSoft);
    if (ui.focusedField == ActiveInput::END_INPUT) DrawRectangleRoundedLines(inE, 0.1f, 4, amberSoft);
    if (ui.focusedField == ActiveInput::START_INPUT) CaptureStringInput(ui.startInputStr);
    if (ui.focusedField == ActiveInput::END_INPUT) CaptureStringInput(ui.endInputStr);

    if (ui.startInputStr.empty()) DrawText("Click building or enter Start...", (int)inS.x + 15, (int)inS.y + 14, 14, GRAY);
    else DrawTextEx(g_hudFont, ui.startInputStr.c_str(), Vector2{ inS.x + 15, inS.y + 14 }, 14, 1, textMain);
    if (ui.endInputStr.empty()) DrawText("Click building or enter Target...", (int)inE.x + 15, (int)inE.y + 14, 14, GRAY);
    else DrawTextEx(g_hudFont, ui.endInputStr.c_str(), Vector2{ inE.x + 15, inE.y + 14 }, 14, 1, textMain);

    Rectangle btn = { ui.menuPos.x + 20, ui.menuPos.y + 250, ui.menuSize.x - 40, 48 }; bool hBtn = CheckCollisionPointRec(m, btn);
    DrawRectangleRounded(btn, 0.1f, 4, hBtn ? Color{ 0, 195, 115, 255 } : Color{ 0, 155, 95, 255 });
    DrawText("COMPUTE ROUTE", (int)btn.x + 150, (int)btn.y + 15, 15, WHITE);
    if (hBtn && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) ui.needsCalculation = true;
    DrawLine(static_cast<int>(ui.menuPos.x) + 20, static_cast<int>(ui.menuPos.y) + 315, static_cast<int>(ui.menuPos.x + ui.menuSize.x) - 20, static_cast<int>(ui.menuPos.y) + 315, Color{ 70, 74, 86, 255 });

    int tableY = (int)ui.menuPos.y + 325;
    if (ui.activeAlgorithm == AlgorithmType::BENCHMARK_COMPARE && !ui.benchResults.empty()) {
        DrawText("ALGORITHM COMPARE MATRIX:", (int)ui.menuPos.x + 20, tableY, 13, SKYBLUE);
        DrawText("Alg Type      | Latency    | Distance", (int)ui.menuPos.x + 20, tableY + 20, 12, GRAY);
        DrawLine((int)ui.menuPos.x + 20, tableY + 35, (int)ui.menuPos.x + (int)ui.menuSize.x - 20, tableY + 35, DARKGRAY);

        auto drawRow = [&](std::string name, int id, int offset, Color c) {
            auto res = ui.benchResults[id];
            std::string tStr = res.success ? TextFormat("%.4f ms", res.timeMs) : "NO PATH";
            std::string dStr = res.success ? TextFormat("%.1f m", res.distance) : "FAILED";
            DrawText(name.c_str(), (int)ui.menuPos.x + 20, tableY + offset, 12, c);
            DrawText(tStr.c_str(), (int)ui.menuPos.x + 160, tableY + offset, 12, c);
            DrawText(dStr.c_str(), (int)ui.menuPos.x + 300, tableY + offset, 12, c);
            };
        drawRow("A* Search     ", 0, 45, GOLD);
        drawRow("Dijkstra Wave ", 1, 65, SKYBLUE);
        drawRow("Greedy TSP    ", 2, 85, LIME);
        tableY += 105;
    }
    else if (ui.isPathFound) {
        DrawText(TextFormat("Total Distance: %.2f m.", ui.routeLength), (int)ui.menuPos.x + 20, tableY + 5, 14, textMain);
        DrawText(TextFormat("Execution Time: %.4f ms", ui.executionTime), (int)ui.menuPos.x + 20, tableY + 25, 14, LIME);
        tableY += 50;
    }
    DrawRectangle(static_cast<int>(ui.menuPos.x) + 20, tableY + 10, static_cast<int>(ui.menuSize.x) - 40, 95, Color{ 20, 21, 26, 255 });
    int cLogY = tableY + 15;
    for (const auto& log : ui.internalLogs) { DrawText(log.text.c_str(), (int)ui.menuPos.x + 32, cLogY, 11, log.color); cLogY += 19; }
}
int main(int argc, char** argv) {
    const int wW = 1900; const int wH = 1000;
    std::unordered_map<long long, Node> parkNodes; std::unordered_map<long long, std::vector<Edge>> parkGraph; bool isDemo = false;
    MapParser mapParser(argc > 1 ? argv[1] : "testpark.osm");

    float mapSizeX = 8000.0f; float mapSizeY = 6000.0f;
    if (mapParser.loadMap()) {
        parkNodes = mapParser.getNodes(); parkGraph = mapParser.getGraph();
        ScaleGraphTo3DSpace(parkNodes, mapParser.min_lat, mapParser.max_lat, mapParser.min_lon, mapParser.max_lon, mapSizeX, mapSizeY);
    }
    else {
        isDemo = true;
        for (int r = 0; r < 7; ++r) {
            for (int c = 0; c < 7; ++c) {
                long long id = r * 7 + c + 1;
                std::string n = (id == 1) ? "Main Gate" : (id == 25) ? "Central Fountain" : (id == 43) ? "Office Building" : (id == 49) ? "Storage Facility" : "";
                parkNodes[id] = Node{ id, 55.7035 + r * 0.001, 37.5150 + c * 0.003, 0.0f, 0.0f, n };
            }
        }
        ScaleGraphTo3DSpace(parkNodes, 55.7035, 55.7095, 37.5150, 37.5330, mapSizeX, mapSizeY);
        auto connect = [&](long long from, long long to) {
            const auto& a = parkNodes.at(from); const auto& b = parkNodes.at(to);
            double distance = GeoDistanceMeters(a.lat, a.lon, b.lat, b.lon);
            parkGraph[from].push_back({to, distance, "path"});
            parkGraph[to].push_back({from, distance, "path"});
        };
        for (int r = 0; r < 7; ++r) {
            for (int c = 0; c < 7; ++c) {
                long long curId = r * 7 + c + 1;
                if (c < 6) connect(curId, curId + 1);
                if (r < 6) connect(curId, curId + 7);
            }
        }
    }

    UIState ui; InitWindow(wW, wH, "GraphNavigator PRO - Stable 3D Engine"); SetTargetFPS(60);
    int cpc = 0; int* cp = LoadCodepoints("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzАБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдеёжзийклмнопрстуфхцчшщъыьэюя1234567890.:- *()[]м, \"«»'–", &cpc);
    g_hudFont = LoadFontEx("C:\\Windows\\Fonts\\arial.ttf", 32, cp, cpc); UnloadCodepoints(cp);
    ui.LogEvent("Core Systems Active. True 3D Viewport Online.", LIME);

    Camera3D camera = { 0 }; camera.position = Vector3{ 0.0f, 7000.0f, 7000.0f }; camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f }; camera.fovy = 45.0f; camera.projection = CAMERA_PERSPECTIVE;
    Navigator navigator(parkNodes, parkGraph);

    while (!WindowShouldClose()) {
        Vector2 m = GetMousePosition();
        if (ui.currentState == AppState::INITIAL_CHOOSE) {
            BeginDrawing(); ClearBackground(Color{ 18, 19, 23, 255 });
            DrawRectangle(wW / 2 - 280, wH / 2 - 240, 560, 480, Color{ 28, 29, 33, 255 });
            DrawRectangleLines(wW / 2 - 280, wH / 2 - 240, 560, 480, Color{ 60, 64, 76, 255 });
            DrawText("SELECT COMPUTATIONAL ALGORITHM MODE", wW / 2 - 235, wH / 2 - 200, 14, WHITE);
            Rectangle r1 = { (float)wW / 2 - 240, (float)wH / 2 - 130, 480, 50 };
            Rectangle r2 = { (float)wW / 2 - 240, (float)wH / 2 - 70, 480, 50 };
            Rectangle r3 = { (float)wW / 2 - 240, (float)wH / 2 - 10, 480, 50 };
            Rectangle r4 = { (float)wW / 2 - 240, (float)wH / 2 + 50, 480, 50 };
            DrawRectangleRounded(r1, 0.1f, 4, CheckCollisionPointRec(m, r1) ? Color{ 254, 215, 60, 255 } : Color{ 160, 130, 30, 255 });
            DrawRectangleRounded(r2, 0.1f, 4, CheckCollisionPointRec(m, r2) ? Color{ 0, 195, 255, 255 } : Color{ 0, 120, 170, 255 });
            DrawRectangleRounded(r3, 0.1f, 4, CheckCollisionPointRec(m, r3) ? Color{ 0, 230, 120, 255 } : Color{ 0, 150, 75, 255 });
            DrawRectangleRounded(r4, 0.1f, 4, CheckCollisionPointRec(m, r4) ? Color{ 255, 110, 110, 255 } : Color{ 180, 50, 50, 255 });
            DrawText("A* Optimization (Custom Heuristic)", (int)r1.x + 85, (int)r1.y + 18, 14, CheckCollisionPointRec(m, r1) ? BLACK : WHITE);
            DrawText("Multi-Source Dijkstra Engine", (int)r2.x + 115, (int)r2.y + 18, 14, WHITE);
            DrawText("Greedy Traveling Salesperson (TSP)", (int)r3.x + 95, (int)r3.y + 18, 14, WHITE);
            DrawText("[ BENCHMARK & COMPARE MULTI-AI MODE ]", (int)r4.x + 75, (int)r4.y + 18, 14, WHITE);
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                if (CheckCollisionPointRec(m, r1)) { ui.activeAlgorithm = AlgorithmType::A_STAR; ui.currentState = AppState::ACTIVE_MAP; }
                if (CheckCollisionPointRec(m, r2)) { ui.activeAlgorithm = AlgorithmType::DIJKSTRA; ui.currentState = AppState::ACTIVE_MAP; }
                if (CheckCollisionPointRec(m, r3)) { ui.activeAlgorithm = AlgorithmType::GREEDY_TSP; ui.currentState = AppState::ACTIVE_MAP; }
                if (CheckCollisionPointRec(m, r4)) { ui.activeAlgorithm = AlgorithmType::BENCHMARK_COMPARE; ui.currentState = AppState::ACTIVE_MAP; }
            }
            EndDrawing();
        }
        else {
            Rectangle totalMenuRect = { ui.menuPos.x, ui.menuPos.y, ui.menuSize.x, ui.menuSize.y };
            if (!CheckCollisionPointRec(m, totalMenuRect)) {
                float wheel = GetMouseWheelMove();
                if (wheel != 0) {
                    float oldHeight = camera.position.y - camera.target.y;
                    float newHeight = std::clamp(oldHeight * std::pow(0.9f, wheel), 50.0f, 12000.0f);
                    float factor = newHeight / oldHeight;
                    camera.position.x = camera.target.x + (camera.position.x - camera.target.x) * factor;
                    camera.position.y = camera.target.y + newHeight;
                    camera.position.z = camera.target.z + (camera.position.z - camera.target.z) * factor;
                }
                if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) ui.dragOffset = m;
                if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                    float moveSpeed = 1.6f;
                    camera.position.x -= (m.x - ui.dragOffset.x) * moveSpeed; camera.target.x -= (m.x - ui.dragOffset.x) * moveSpeed;
                    camera.position.z -= (m.y - ui.dragOffset.y) * moveSpeed; camera.target.z -= (m.y - ui.dragOffset.y) * moveSpeed;
                    ui.dragOffset = m;
                }
            }
            Rectangle hRect = { ui.menuPos.x, ui.menuPos.y, ui.menuSize.x, 55.0f };
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(m, hRect)) { ui.isDragging = true; ui.dragOffset = { m.x - ui.menuPos.x, m.y - ui.menuPos.y }; }
            if (ui.isDragging) { ui.menuPos.x = m.x - ui.dragOffset.x; ui.menuPos.y = m.y - ui.dragOffset.y; if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) ui.isDragging = false; }

            if (ui.needsCalculation) {
                ui.needsCalculation = false; ui.clickedNodeIds.clear();
                ui.currentRoute = {}; ui.isPathFound = false; ui.benchResults.clear();
                ui.executionTime = 0; ui.routeLength = 0;
                long long sId = FindValidRouterNode(parkNodes, parkGraph, ui.startInputStr);
                long long eId = FindValidRouterNode(parkNodes, parkGraph, ui.endInputStr);
                if (sId != -1 && eId != -1) {
                    ui.clickedNodeIds.push_back(sId); ui.clickedNodeIds.push_back(eId);
                    if (ui.activeAlgorithm == AlgorithmType::BENCHMARK_COMPARE) {
                        ShortestPath rA = navigator.findPathAStar(sId, eId);
                        ShortestPath rD = navigator.findPathMultiSource({ sId }, eId);
                        ShortestPath rG = navigator.findPathGreedyTSP(ui.clickedNodeIds);

                        // ИСПРАВЛЕНО: Безопасная инициализация хэш-карты результатов бенчмарка с явными ID ключами
                        ui.benchResults[0] = AlgResult{ rA.execution_time_ms, rA.total_length, rA.success };
                        ui.benchResults[1] = AlgResult{ rD.execution_time_ms, rD.total_length, rD.success };
                        ui.benchResults[2] = AlgResult{ rG.execution_time_ms, rG.total_length, rG.success };

                        ui.currentRoute = rA; ui.isPathFound = rA.success;
                        if (ui.isPathFound) { ui.executionTime = rA.execution_time_ms; ui.routeLength = rA.total_length; ui.LogEvent("Benchmark test completed.", LIME); }
                    }
                    else {
                        if (ui.activeAlgorithm == AlgorithmType::A_STAR) ui.currentRoute = navigator.findPathAStar(sId, eId);
                        else if (ui.activeAlgorithm == AlgorithmType::DIJKSTRA) ui.currentRoute = navigator.findPathMultiSource({ sId }, eId);
                        else ui.currentRoute = navigator.findPathGreedyTSP(ui.clickedNodeIds);
                        ui.isPathFound = ui.currentRoute.success;
                        if (ui.isPathFound) { ui.executionTime = ui.currentRoute.execution_time_ms; ui.routeLength = ui.currentRoute.total_length; ui.LogEvent("Route OK.", LIME); }
                    }
                }
                else ui.LogEvent("Object name not found in graph.", RED);
                if (sId != -1 && eId != -1 && !ui.isPathFound)
                    ui.LogEvent("No route connects the selected objects.", RED);
            }
            if (IsKeyPressed(KEY_C) && ui.focusedField == ActiveInput::NONE) { ui.startInputStr = ""; ui.endInputStr = ""; ui.isPathFound = false; ui.benchResults.clear(); }

            BeginDrawing(); ClearBackground(Color{ 20, 22, 28, 255 });
            BeginMode3D(camera);
            DrawPlane(Vector3{ 0.0f, 0.0f, 0.0f }, Vector2{ 10000.0f, 10000.0f }, Color{ 46, 125, 50, 255 });
            for (const auto& [sourceId, edges] : parkGraph) {
                auto srcIt = parkNodes.find(sourceId); if (srcIt == parkNodes.end()) continue;
                Vector3 pSrc = { static_cast<float>(srcIt->second.x), 1.5f, static_cast<float>(srcIt->second.y) };
                for (const Edge& edge : edges) {
                    auto dstIt = parkNodes.find(edge.to); if (dstIt == parkNodes.end()) continue;
                    Vector3 pDst = { static_cast<float>(dstIt->second.x), 1.5f, static_cast<float>(dstIt->second.y) };
                    DrawLine3D(pSrc, pDst, Color{ 140, 145, 155, 255 });
                    DrawLine3D(Vector3{ pSrc.x, 2.0f, pSrc.z }, Vector3{ pDst.x, 2.0f, pDst.z }, Color{ 160, 165, 175, 255 });
                }
            }
            Ray ray = GetMouseRay(m, camera);
            std::unordered_set<std::string> rendered3DNames;
            for (const auto& [id, node] : parkNodes) {
                if (node.name.empty() || node.name.find("проспект") != std::string::npos || node.name.find("улица") != std::string::npos || node.name.find("шоссе") != std::string::npos) continue;
                if (rendered3DNames.find(node.name) != rendered3DNames.end()) continue;
                float bSize = 8.0f; float bHeight = 18.0f; Vector3 bPos = { static_cast<float>(node.x), bHeight / 2.0f, static_cast<float>(node.y) };
                RayCollision boxHit = GetRayCollisionBox(ray, BoundingBox{ {bPos.x - bSize / 2, 0, bPos.z - bSize / 2}, {bPos.x + bSize / 2, bHeight, bPos.z + bSize / 2} });
                if (boxHit.hit && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !CheckCollisionPointRec(m, totalMenuRect)) {
                    if (ui.focusedField == ActiveInput::START_INPUT) ui.startInputStr = node.name;
                    if (ui.focusedField == ActiveInput::END_INPUT) ui.endInputStr = node.name;
                }
                DrawCube(bPos, bSize, bHeight, bSize, Color{ 60, 65, 75, 255 }); DrawCubeWires(bPos, bSize, bHeight, bSize, Color{ 180, 185, 200, 255 });
                rendered3DNames.insert(node.name);
            }
            if (ui.isPathFound && ui.currentRoute.node_ids.size() > 1) {
                for (size_t i = 0; i < ui.currentRoute.node_ids.size() - 1; ++i) {
                    auto it1 = parkNodes.find(ui.currentRoute.node_ids[i]); auto it2 = parkNodes.find(ui.currentRoute.node_ids[i + 1]);
                    if (it1 != parkNodes.end() && it2 != parkNodes.end()) {
                        Vector3 p1 = { static_cast<float>(it1->second.x), 3.0f, static_cast<float>(it1->second.y) };
                        Vector3 p2 = { static_cast<float>(it2->second.x), 3.0f, static_cast<float>(it2->second.y) };
                        DrawLine3D(p1, p2, Color{ 255, 170, 0, 255 }); DrawLine3D(Vector3{ p1.x, 3.5f, p1.z }, Vector3{ p2.x, 3.5f, p2.z }, Color{ 255, 235, 0, 255 });
                    }
                }
            }
            EndMode3D();
            std::unordered_set<std::string> rendered2DNames; int staggeredRow = 0;
            for (const auto& [id, node] : parkNodes) {
                if (node.name.empty() || node.name.find("проспект") != std::string::npos || node.name.find("улица") != std::string::npos || node.name.find("шоссе") != std::string::npos) continue;
                if (rendered2DNames.find(node.name) != rendered2DNames.end()) continue;
                float bHeight = 18.0f; float dynamicHeightOffset = 15.0f + (staggeredRow % 4) * 20.0f; staggeredRow++;
                Vector2 sPos = GetWorldToScreen(Vector3{ static_cast<float>(node.x), bHeight + dynamicHeightOffset, static_cast<float>(node.y) }, camera);
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !CheckCollisionPointRec(m, totalMenuRect)) {
                    if (CheckCollisionCircles(m, 1.0f, sPos, 55.0f)) {
                        if (ui.focusedField == ActiveInput::START_INPUT) ui.startInputStr = node.name;
                        if (ui.focusedField == ActiveInput::END_INPUT) ui.endInputStr = node.name;
                    }
                }
                DrawRectangle((int)sPos.x - 45, (int)sPos.y - 8, 90, 16, Fade(Color{ 20, 22, 28, 255 }, 0.8f));
                DrawTextEx(g_hudFont, node.name.c_str(), Vector2{ sPos.x - 40, sPos.y - 5 }, 10, 1, Color{ 0, 255, 200, 255 });
                rendered2DNames.insert(node.name);
            }
            DrawControlHUD(ui, parkNodes.size(), isDemo); EndDrawing();
        }
    }
    UnloadFont(g_hudFont); CloseWindow(); return 0;
}
