"""mapbake: превращает OSM XML (.osm) в бинарный пакет города (.gnpk).

Прототип на стандартной библиотеке Python (iterparse читает файл потоково,
поэтому большой район не грузится в память целиком). Позже логику можно
перенести на C++/libosmium для .pbf всей Москвы.

Формат пакета v1 (little-endian), его же читает client/data/Package.cpp:
  magic "GNPK", u32 version, f64 origin_lat, f64 origin_lon
  u32 node_count, u32 edge_count, u32 building_count, u32 tree_count,
  u32 road_count, u32 crossing_count, u32 name_count
  nodes:     node_count * (f32 x, f32 z)           -- метры от origin
  offsets:   (node_count+1) * u32                  -- CSR
  edges:     edge_count * (u32 to, f32 w, u8 type)
  buildings: (u32 n, f32 height, n*(f32 x, f32 z))
  trees:     tree_count * (f32 x, f32 z)
  roads:     (u8 kind, f32 width, u32 n, n*(f32 x, f32 z))
  crossings: crossing_count * u32 node_index
  names:     (u32 node_index, u16 len, utf8 bytes)

Использование: python mapbake.py input.osm output.gnpk
"""
import math
import struct
import sys
import xml.etree.ElementTree as ET

# Пешеходные дороги: тот же фильтр, что был в MapParser.cpp.
WALKABLE = {"footway": 1, "path": 1, "pedestrian": 2, "steps": 3, "track": 1,
            "cycleway": 1, "living_street": 4, "service": 4,
            "residential": 4, "unclassified": 4}
# Дороги, которые рисуем лентами (вид ленты, ширина в метрах).
ROAD_STYLE = {"motorway": (5, 22), "trunk": (5, 18), "primary": (5, 14),
              "secondary": (4, 11), "tertiary": (4, 9), "residential": (4, 7),
              "unclassified": (4, 6), "living_street": (4, 6), "service": (4, 4),
              "pedestrian": (2, 6), "footway": (1, 2.5), "path": (1, 2),
              "cycleway": (1, 2.5), "steps": (3, 2.5), "track": (1, 3)}
EARTH_R = 6371000.0


def walk_reject(tags):
    """'' если путь пешеходный, иначе причина отказа."""
    if tags.get("highway") not in WALKABLE:
        return "not walkable"
    if tags.get("area") == "yes":
        return "area"
    foot = tags.get("foot", "")
    if foot in ("no", "private"):
        return "foot restricted"
    if foot not in ("yes", "designated", "permissive") and tags.get("access") in ("no", "private"):
        return "access restricted"
    return ""


def building_height(tags):
    """Высота здания: height, иначе этажи * 3 м, иначе 10 м."""
    for key in ("height", "building:height"):
        try:
            return max(2.0, float(tags[key].replace("m", "").strip()))
        except (KeyError, ValueError):
            pass
    try:
        return max(3.0, float(tags["building:levels"]) * 3.0)
    except (KeyError, ValueError):
        return 10.0


def haversine(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(h)))


def bake(src, dst):
    coords, node_tags, ways = {}, {}, []
    for _, el in ET.iterparse(src, events=("end",)):
        if el.tag == "node":
            lat, lon = float(el.get("lat")), float(el.get("lon"))
            if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90 and abs(lon) <= 180):
                raise ValueError("bad coordinates in node " + el.get("id"))
            nid = int(el.get("id"))
            coords[nid] = (lat, lon)
            tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
            if tags:
                node_tags[nid] = tags
            el.clear()
        elif el.tag == "way":
            ways.append(([int(n.get("ref")) for n in el.findall("nd")],
                         {t.get("k"): t.get("v") for t in el.findall("tag")}))
            el.clear()
    if not coords:
        raise ValueError("no nodes")

    lats = [c[0] for c in coords.values()]
    lons = [c[1] for c in coords.values()]
    olat, olon = (min(lats) + max(lats)) / 2, (min(lons) + max(lons)) / 2
    kx = math.radians(1) * EARTH_R * math.cos(math.radians(olat))
    kz = math.radians(1) * EARTH_R

    def local(nid):  # север = -Z, как в старом клиенте
        lat, lon = coords[nid]
        return ((lon - olon) * kx, -(lat - olat) * kz)

    index, adj, buildings, roads = {}, [], [], []

    def idx(nid):
        if nid not in index:
            index[nid] = len(adj)
            adj.append([])
        return index[nid]

    for refs, tags in ways:
        refs = [r for r in refs if r in coords]
        if len(refs) < 2:
            continue
        if "building" in tags and len(refs) >= 4 and refs[0] == refs[-1]:
            buildings.append((building_height(tags), [local(r) for r in refs[:-1]]))
            continue
        hw = tags.get("highway")
        if hw in ROAD_STYLE and tags.get("area") != "yes":
            kind, width = ROAD_STYLE[hw]
            if tags.get("footway") == "crossing":
                kind = 6
            roads.append((kind, float(width), [local(r) for r in refs]))
        if hw and not walk_reject(tags):
            t = WALKABLE[hw]
            for a, b in zip(refs, refs[1:]):
                if a == b:
                    continue
                w = haversine(coords[a], coords[b])
                ia, ib = idx(a), idx(b)
                adj[ia].append((ib, w, t))
                adj[ib].append((ia, w, t))

    by_index = sorted(index.items(), key=lambda kv: kv[1])
    trees = [local(n) for n, t in node_tags.items() if t.get("natural") == "tree"]
    crossings = [index[n] for n, t in node_tags.items()
                 if n in index and (t.get("highway") == "crossing" or t.get("footway") == "crossing")]
    names = [(index[n], t["name"].encode("utf-8")[:65535]) for n, t in node_tags.items()
             if n in index and t.get("name")]

    out = bytearray(b"GNPK")
    out += struct.pack("<Idd7I", 1, olat, olon, len(adj), sum(map(len, adj)),
                       len(buildings), len(trees), len(roads), len(crossings), len(names))
    for nid, _ in by_index:
        out += struct.pack("<ff", *local(nid))
    off = 0
    for edges in adj + [[]]:
        out += struct.pack("<I", off)
        off += len(edges)
    for edges in adj:
        for to, w, t in edges:
            out += struct.pack("<IfB", to, w, t)
    for h, pts in buildings:
        out += struct.pack("<If", len(pts), h)
        for p in pts:
            out += struct.pack("<ff", *p)
    for p in trees:
        out += struct.pack("<ff", *p)
    for kind, width, pts in roads:
        out += struct.pack("<BfI", kind, width, len(pts))
        for p in pts:
            out += struct.pack("<ff", *p)
    for c in crossings:
        out += struct.pack("<I", c)
    for n, raw in names:
        out += struct.pack("<IH", n, len(raw)) + raw
    with open(dst, "wb") as f:
        f.write(out)
    print(f"nodes={len(adj)} buildings={len(buildings)} trees={len(trees)} "
          f"roads={len(roads)} crossings={len(crossings)} names={len(names)} bytes={len(out)}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: mapbake.py input.osm output.gnpk")
    bake(sys.argv[1], sys.argv[2])
