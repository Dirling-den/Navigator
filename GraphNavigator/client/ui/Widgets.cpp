#include "Widgets.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <vector>

namespace gn::ui {
namespace {
Font g_regular{}, g_bold{};
bool g_loaded = false, g_wantHand = false, g_wantBeam = false, g_overUi = false, g_prevOverUi = false;

Font loadOne(const char* primary, const char* fallback) {
    // Латиница, кириллица, типографские символы и стрелки для иконок.
    std::vector<int> cps;
    for (int c = 32; c < 127; ++c) cps.push_back(c);
    for (int c = 0x400; c <= 0x45F; ++c) cps.push_back(c);
    for (int c : { 0xAB, 0xBB, 0xB7, 0xD7, 0x2013, 0x2014, 0x2026, 0x2022, 0x2190, 0x2191, 0x2192, 0x2193, 0x21C5, 0x2212, 0x2715 })
        cps.push_back(c);
    const char* path = FileExists(primary) ? primary : fallback;
    Font f = LoadFontEx(path, 48, cps.data(), int(cps.size()));
    SetTextureFilter(f.texture, TEXTURE_FILTER_BILINEAR);
    return f;
}
} // namespace

void loadFonts() {
    if (g_loaded) return;
    g_regular = loadOne("C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf");
    g_bold = loadOne("C:/Windows/Fonts/segoeuib.ttf", "C:/Windows/Fonts/arialbd.ttf");
    g_loaded = true;
}
void unloadFonts() { if (g_loaded) { UnloadFont(g_regular); UnloadFont(g_bold); g_loaded = false; } }
Font& font(bool bold) { return bold ? g_bold : g_regular; }

void text(const std::string& s, float x, float y, float size, Color c, bool bold) {
    DrawTextEx(font(bold), s.c_str(), { std::round(x), std::round(y) }, size, 0, c);
}
float textWidth(const std::string& s, float size, bool bold) { return MeasureTextEx(font(bold), s.c_str(), size, 0).x; }
void textCentered(const std::string& s, Rectangle r, float size, Color c, bool bold) {
    Vector2 m = MeasureTextEx(font(bold), s.c_str(), size, 0);
    text(s, r.x + (r.width - m.x) / 2, r.y + (r.height - m.y) / 2, size, c, bold);
}

void blockMouse(Rectangle r) { if (CheckCollisionPointRec(GetMousePosition(), r)) g_overUi = true; }
bool mouseOverUi() { return g_overUi || g_prevOverUi; }
void beginFrame() { g_wantHand = g_wantBeam = false; g_prevOverUi = g_overUi; g_overUi = false; }
void endFrame() { SetMouseCursor(g_wantBeam ? MOUSE_CURSOR_IBEAM : g_wantHand ? MOUSE_CURSOR_POINTING_HAND : MOUSE_CURSOR_DEFAULT); }

void card(Rectangle r, float radius) {
    float rr = radius / std::min(r.width, r.height);
    for (int i = 3; i >= 1; --i)   // мягкая тень из нескольких полупрозрачных слоёв
        DrawRectangleRounded({ r.x - i, r.y - i + 2.0f * i, r.width + 2.0f * i, r.height + 2.0f * i }, rr, 8, Color{ 0, 0, 0, (unsigned char)(10 - i * 2) });
    DrawRectangleRounded(r, rr, 8, theme::card);
    blockMouse(r);
}
bool hovered(Rectangle r) { return CheckCollisionPointRec(GetMousePosition(), r); }

bool button(Rectangle r, const std::string& label, bool primary, bool enabled) {
    bool h = enabled && hovered(r);
    if (h) g_wantHand = true;
    float rr = 0.5f;
    if (primary) DrawRectangleRounded(r, rr, 8, !enabled ? Color{ 180, 200, 230, 255 } : h ? theme::accentHover : theme::accent);
    else { DrawRectangleRounded(r, rr, 8, h ? theme::hover : theme::card); DrawRectangleRoundedLines(r, rr, 8, theme::border); }
    textCentered(label, r, 17, primary ? WHITE : (enabled ? theme::accent : theme::muted), true);
    blockMouse(r);
    return h && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

bool iconButton(Vector2 c, float radius, const std::string& glyph) {
    bool h = CheckCollisionPointCircle(GetMousePosition(), c, radius);
    if (h) g_wantHand = true;
    DrawCircleV({ c.x, c.y + 2 }, radius + 1, Color{ 0, 0, 0, 22 });
    DrawCircleV(c, radius, h ? theme::hover : theme::card);
    textCentered(glyph, { c.x - radius, c.y - radius - 1, radius * 2, radius * 2 }, radius, theme::text, true);
    blockMouse({ c.x - radius, c.y - radius, radius * 2, radius * 2 });
    return h && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

bool segment(Rectangle r, const std::string& label, bool active) {
    bool h = hovered(r);
    if (h) g_wantHand = true;
    if (active) DrawRectangleRounded(r, 0.5f, 8, theme::accentSoft);
    else if (h) DrawRectangleRounded(r, 0.5f, 8, theme::hover);
    textCentered(label, r, 15, active ? theme::accent : theme::muted, true);
    return h && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

void captureUtf8(std::string& s) {
    for (int key = GetCharPressed(); key > 0; key = GetCharPressed()) {
        if (key < 32 || key == 127) continue;
        int len = 0; const char* u = CodepointToUTF8(key, &len);
        if (s.size() + len < 200) s.append(u, len);
    }
    if ((IsKeyPressed(KEY_BACKSPACE) || IsKeyPressedRepeat(KEY_BACKSPACE)) && !s.empty()) {
        while (!s.empty() && (static_cast<unsigned char>(s.back()) & 0xC0) == 0x80) s.pop_back();
        if (!s.empty()) s.pop_back();
    }
}

bool textField(Rectangle r, std::string& value, const std::string& placeholder, bool focused, Color dot) {
    bool h = hovered(r);
    DrawRectangleRounded(r, 0.25f, 8, focused ? theme::card : theme::hover);
    if (focused) DrawRectangleRoundedLinesEx(r, 0.25f, 8, 2, theme::accent);
    DrawCircleV({ r.x + 18, r.y + r.height / 2 }, 5, dot);
    float tx = r.x + 34, ty = r.y + (r.height - 18) / 2;
    if (focused) captureUtf8(value);
    // Длинный текст обрезаем слева, чтобы курсор был виден.
    std::string shown = value;
    while (!shown.empty() && textWidth(shown, 18) > r.width - 50) {
        size_t i = 1; while (i < shown.size() && (static_cast<unsigned char>(shown[i]) & 0xC0) == 0x80) ++i;
        shown.erase(0, i);
    }
    if (value.empty()) text(placeholder, tx, ty, 18, theme::muted);
    else text(shown, tx, ty, 18, theme::text);
    if (focused && std::fmod(GetTime(), 1.0) < 0.55)
        DrawRectangle(int(tx + textWidth(shown, 18) + 1), int(ty + 1), 2, 18, theme::accent);
    if (h) g_wantBeam = true;
    blockMouse(r);
    return h && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

std::string normalize(const std::string& s) {
    std::string out;
    for (size_t i = 0; i < s.size();) {
        int bytes = 0;
        int cp = GetCodepointNext(s.c_str() + i, &bytes);
        i += size_t(std::max(bytes, 1));
        if (cp < 128 && std::isspace(cp)) continue;
        if (cp >= 'A' && cp <= 'Z') cp += 32;
        if (cp >= 0x410 && cp <= 0x42F) cp += 0x20;
        if (cp == 0x401 || cp == 0x451) cp = 0x435;           // ё → е
        int len = 0; const char* u = CodepointToUTF8(cp, &len);
        out.append(u, len);
    }
    return out;
}

} // namespace gn::ui
