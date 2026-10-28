"""Генерирует ДЕМО-район (не реальные данные!) в формате OSM XML для проверки
mapbake и клиента без скачивания OSM: сетка улиц, кварталы со зданиями разной
высоты, тротуары, переходы-зебры, парк с деревьями и тропинками.

Для настоящей Москвы скачайте выгрузку (например, с download.geofabrik.de
или через overpass/JOSM) и передайте .osm в mapbake.py.
Использование: python make_demo_osm.py out.osm
"""
import math
import random
import sys

random.seed(7)
LAT0, LON0 = 55.7520, 37.6175
M_LAT = 1 / 111195.0
M_LON = 1 / (111195.0 * math.cos(math.radians(LAT0)))

nodes, ways = [], []
_nid = [0]


def node(x, z, tags=None):
    _nid[0] += 1
    nodes.append((_nid[0], LAT0 - z * M_LAT, LON0 + x * M_LON, tags or {}))
    return _nid[0]


def way(refs, tags):
    ways.append((refs, tags))


GRID, BLOCK = 6, 160            # 6x6 перекрёстков, квартал 160 м
cross = {}
for i in range(GRID):
    for j in range(GRID):
        x, z = (i - GRID / 2) * BLOCK, (j - GRID / 2) * BLOCK
        tags = {"highway": "crossing"} if (i + j) % 2 == 0 else {}
        cross[i, j] = node(x, z, tags)
names = ["Тверская улица", "Улица Арбат", "Пушечная улица", "Неглинная улица", "Улица Петровка", "Улица Рождественка"]
for i in range(GRID):
    way([cross[i, j] for j in range(GRID)], {"highway": "secondary" if i % 3 == 0 else "residential", "name": names[i]})
    way([cross[j, i] for j in range(GRID)], {"highway": "tertiary", "name": names[-1 - i]})
# Переходы-зебры через улицы у перекрёстков
for (i, j), n in cross.items():
    if i + 1 < GRID and (i + j) % 2 == 0:
        x, z = (i - GRID / 2) * BLOCK + 14, (j - GRID / 2) * BLOCK
        a, b = node(x, z - 7), node(x, z + 7)
        way([a, b], {"highway": "footway", "footway": "crossing"})
        way([n, a], {"highway": "footway"})
        way([b, n], {"highway": "footway"})

places = ["Главный вход", "Фонтан", "Библиотека", "Кафе «Сквер»", "Музей", "Школа №1", "Поликлиника", "Станция метро"]
for i in range(GRID - 1):
    for j in range(GRID - 1):
        x0, z0 = (i - GRID / 2) * BLOCK + 14, (j - GRID / 2) * BLOCK + 14
        size = BLOCK - 28
        if (i, j) == (2, 2):     # парк: тропинки и деревья
            c = node(x0 + size / 2, z0 + size / 2, {"name": "Фонтан"})
            for k, (dx, dz) in enumerate([(0, 0), (size, 0), (size, size), (0, size)]):
                corner = node(x0 + dx, z0 + dz)
                way([corner, c], {"highway": "footway"})
                way([cross[i + (dx > 0), j + (dz > 0)], corner], {"highway": "path"})
            for _ in range(70):
                node(x0 + random.uniform(4, size - 4), z0 + random.uniform(4, size - 4), {"natural": "tree"})
            continue
        # Дворовый проход через квартал с названием места
        mid = node(x0 + size / 2, z0 + size / 2, {"name": places[(i * 5 + j) % len(places)]} if (i + j) % 3 == 0 else {})
        way([cross[i, j], mid, cross[i + 1, j + 1]], {"highway": "footway"})
        # 4 здания по углам квартала
        for qx in (0, 1):
            for qz in (0, 1):
                bx, bz, bw = x0 + qx * size * 0.55 + 4, z0 + qz * size * 0.55 + 4, size * 0.4
                levels = random.choice([3, 5, 5, 9, 12, 16, 22])
                shape = [(0, 0), (bw, 0), (bw, bw), (0, bw)]
                if random.random() < 0.4:   # Г-образное здание
                    shape = [(0, 0), (bw, 0), (bw, bw * 0.45), (bw * 0.45, bw * 0.45), (bw * 0.45, bw), (0, bw)]
                refs = [node(bx + dx, bz + dz) for dx, dz in shape]
                way(refs + refs[:1], {"building": "yes", "building:levels": str(levels)})
        for _ in range(6):   # деревья вдоль улиц
            node(x0 + random.uniform(0, size), z0 - 6, {"natural": "tree"})

with open(sys.argv[1] if len(sys.argv) > 1 else "demo_district.osm", "w", encoding="utf-8") as f:
    f.write('<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6" generator="GraphNavigator demo">\n')
    esc = lambda s: s.replace("&", "&amp;").replace('"', "&quot;").replace("<", "&lt;")
    for nid, lat, lon, tags in nodes:
        f.write(f'<node id="{nid}" lat="{lat:.7f}" lon="{lon:.7f}"')
        f.write(">" + "".join(f'<tag k="{k}" v="{esc(v)}"/>' for k, v in tags.items()) + "</node>\n" if tags else "/>\n")
    for wid, (refs, tags) in enumerate(ways, start=10**6):
        f.write(f'<way id="{wid}">' + "".join(f'<nd ref="{r}"/>' for r in refs)
                + "".join(f'<tag k="{k}" v="{esc(v)}"/>' for k, v in tags.items()) + "</way>\n")
    f.write("</osm>\n")
print(f"demo nodes={len(nodes)} ways={len(ways)}")
