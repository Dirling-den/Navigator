#include "Render.hpp"
#include "raylib.h" // Подключаем легковесную графическую библиотеку

Render::Render(int windowWidth, int windowHeight, const std::string& windowTitle)
    : m_width(windowWidth), m_height(windowHeight)
{
    // Инициализируем графическое окно и ставим стабильные 60 FPS
    InitWindow(m_width, m_height, windowTitle.c_str());
    SetTargetFPS(60);
}

Render::~Render() {
    // Безопасное закрытие окна при уничтожении объекта Render
    CloseWindow();
}

bool Render::shouldClose() const {
    return WindowShouldClose();
}

void Render::drawFrame(const std::unordered_map<long long, Node>& nodes,
    const std::unordered_map<long long, std::vector<Edge>>& graph,
    const std::vector<long long>& activePath)
{
    // Начинаем процесс отрисовки текущего кадра
    BeginDrawing();
    ClearBackground(RAYWHITE); // Очищаем экран мягким белым цветом

    // 1. ОТРИСОВКА ВСЕХ ДОРОЖЕК ПАРКА (РЕБРА ГРАФА)
    // Рисуем обычные ребра тонкими серыми линиями
    for (const auto& [fromId, edges] : graph) {
        auto fromIt = nodes.find(fromId);
        if (fromIt == nodes.end()) continue;

        for (const auto& edge : edges) {
            auto toIt = nodes.find(edge.to);
            if (toIt == nodes.end()) continue;

            // Рисуем линию между экранными координатами (x, y) двух вершин
            DrawLineEx(
                Vector2{ fromIt->second.x, fromIt->second.y },
                Vector2{ toIt->second.x, toIt->second.y },
                2.0f, // Толщина дорожки
                LIGHTGRAY
            );
        }
    }

    // 2. ПОДСВЕТКА НАЙДЕННОГО МАРШРУТА
    // Если навигатор вернул непустой путь, рисуем его жирной синей линией
    if (activePath.size() >= 2) {
        for (size_t i = 0; i < activePath.size() - 1; ++i) {
            auto fromIt = nodes.find(activePath[i]);
            auto toIt = nodes.find(activePath[i + 1]);

            if (fromIt != nodes.end() && toIt != nodes.end()) {
                DrawLineEx(
                    Vector2{ fromIt->second.x, fromIt->second.y },
                    Vector2{ toIt->second.x, toIt->second.y },
                    5.0f, // Маршрут делаем толще обычных дорог
                    BLUE
                );
            }
        }
    }

    // 3. ОТРИСОВКА ПЕРЕКРЕСТКОВ И КЛЮЧЕВЫХ ТОЧЕК (ВЕРШИНЫ ГРАФА)
    for (const auto& [id, node] : nodes) {
        // Базовый цвет точки — темно-серый круг
        Color nodeColor = DARKGRAY;
        float radius = 5.0f;

        // Если у точки есть имя (например, "Фонтан"), делаем её больше и выделяем зеленым
        if (!node.name.empty()) {
            nodeColor = GREEN;
            radius = 8.0f;
            // Рисуем текст названия чуть выше самой точки
            DrawText(node.name.c_str(), (int)node.x - 20, (int)node.y - 20, 12, DARKGRAY);
        }

        DrawCircle((int)node.x, (int)node.y, radius, nodeColor);
    }

    // Инструкция для пользователя в левом верхнем углу экрана
    DrawText("ЛКМ: Выбрать старт | ПКМ: Выбрать финиш", 10, 10, 16, BLACK);

    // Заканчиваем отрисовку и выводим кадр на монитор
    EndDrawing();
}

long long Render::getNodeIdByClick(const std::unordered_map<long long, Node>& nodes, float clickRadius) const {
    // Получаем текущие координаты мышки в пикселях
    Vector2 mousePos = GetMousePosition();

    // Ищем, попала ли мышка в радиус какой-либо вершины
    for (const auto& [id, node] : nodes) {
        float dx = mousePos.x - node.x;
        float dy = mousePos.y - node.y;
        float distanceSquared = dx * dx + dy * dy;

        // Проверка попадания по теореме Пифагора (без медленного извлечения корня)
        if (distanceSquared <= clickRadius * clickRadius) {
            return id; // Возвращаем ID вершины, по которой кликнули
        }
    }
    return -1; // Кликнули в пустое место
}
