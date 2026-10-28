// Widgets.hpp — небольшой набор immediate-mode виджетов в стиле современных
// картографических приложений: карточки с тенью, кнопки, поля ввода, тосты.
#pragma once
#include "raylib.h"
#include <string>

namespace gn::ui {

// Палитра (светлая тема, как у публичных карт).
namespace theme {
inline const Color bg{ 245, 247, 250, 255 };
inline const Color card{ 255, 255, 255, 255 };
inline const Color border{ 222, 226, 232, 255 };
inline const Color text{ 32, 33, 36, 255 };
inline const Color muted{ 112, 117, 125, 255 };
inline const Color accent{ 26, 115, 232, 255 };
inline const Color accentHover{ 21, 98, 204, 255 };
inline const Color accentSoft{ 232, 240, 254, 255 };
inline const Color green{ 52, 168, 83, 255 };
inline const Color red{ 234, 67, 53, 255 };
inline const Color hover{ 241, 243, 244, 255 };
}

// Шрифты загружаются один раз после InitWindow (Segoe UI с кириллицей, запасной — Arial).
void loadFonts();
void unloadFonts();
Font& font(bool bold = false);

void text(const std::string& s, float x, float y, float size, Color c, bool bold = false);
float textWidth(const std::string& s, float size, bool bold = false);
void textCentered(const std::string& s, Rectangle r, float size, Color c, bool bold = false);

void card(Rectangle r, float radius = 12);            // белая карточка с мягкой тенью
bool hovered(Rectangle r);
// Кнопка. primary — залитая синим; иначе — плоская. Возвращает true при клике.
bool button(Rectangle r, const std::string& label, bool primary = false, bool enabled = true);
// Круглая иконка-кнопка (символ в центре).
bool iconButton(Vector2 c, float radius, const std::string& glyph);
// Сегмент переключателя; true при клике.
bool segment(Rectangle r, const std::string& label, bool active);

// Поле ввода: focused управляется снаружи. Возвращает true, если по полю кликнули.
bool textField(Rectangle r, std::string& value, const std::string& placeholder, bool focused, Color dot);
void captureUtf8(std::string& s);                      // ввод с клавиатуры в UTF-8

// Нормализация для поиска без учёта регистра (латиница + кириллица, ё=е, без пробелов).
std::string normalize(const std::string& s);

// Курсор: виджеты запрашивают «руку» над кликабельным.
void beginFrame();
void endFrame();
bool mouseOverUi();                                     // мышь над любым элементом UI в этом кадре
void blockMouse(Rectangle r);                           // отметить область как UI

} // namespace gn::ui
