#pragma once
#include <unordered_map>
#include <vector>
#include "MapData.hpp"

class Render {
public:
    // Конструктор инициализирует размеры окна приложения
    Render(int windowWidth, int windowHeight, const std::string& windowTitle);

    // Деструктор автоматически закрывает графическое окно
    ~Render();

    // Главный метод отрисовки кадра
    // Принимает граф для отрисовки и опциональный найденный путь для подсветки
    void drawFrame(const std::unordered_map<long long, Node>& nodes,
        const std::unordered_map<long long, std::vector<Edge>>& graph,
        const std::vector<long long>& activePath = {});

    // Проверка, закрыл ли пользователь окно
    bool shouldClose() const;

    // Вспомогательный метод для определения, кликнул ли пользователь на вершину графа
    // Возвращает ID вершины или -1, если клик был мимо
    long long getNodeIdByClick(const std::unordered_map<long long, Node>& nodes, float clickRadius = 15.0f) const;

private:
    int m_width;
    int m_height;
};
