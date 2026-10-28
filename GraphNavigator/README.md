# GraphNavigator

C++17, raylib 5.5, tinyxml2 11.0.0. CMake загружает закреплённые версии библиотек при первой конфигурации (нужен интернет).

## Сборка Windows

Установите Visual Studio с компонентом «Разработка классических приложений на C++» и CMake. Откройте эту папку через **Открыть → Папка** в Visual Studio: используется `CMakeLists.txt`, а не пустой старый `GraphNavigator.slnx`.

В Developer PowerShell:

```powershell
cmake -S . -B build -A x64
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
.\build\Debug\GraphNavigator.exe "C:\maps\park.osm"
```

Без аргумента программа ищет `testpark.osm` в рабочей папке. Если файл отсутствует, повреждён или не содержит пешеходных рёбер, открывается демонстрационная карта. Старый `main.exe` не пересобирается и не заменяется — запускайте новый исполняемый файл из `build`.

Для сборки только ядра и тестов: добавьте `-DGRAPHNAVIGATOR_BUILD_APP=OFF` при конфигурации.

## Управление

Выберите алгоритм, затем поле старта/финиша. Введите название или нажмите на объект/подпись на карте. Нажмите `COMPUTE ROUTE`. ПКМ перемещает карту, колесо масштабирует; `C` очищает выбор, если поле ввода не активно. Поля поддерживают UTF-8; поиск не учитывает регистр латиницы и русской кириллицы. Для шрифта в Windows используется Arial.

TSP строит жадный открытый маршрут по переданному списку точек; интерфейс сейчас передаёт две точки. Это не точный коммивояжёр и не замкнутый тур. A* предполагает, что веса рёбер — географические расстояния в метрах (так формируются OSM и демо-граф).

## Новый клиент (3D-город)

```powershell
python mapbake/make_demo_osm.py maps/demo_district.osm      # ДЕМО-район, не реальные данные
python mapbake/mapbake.py maps/demo_district.osm maps/demo_district.gnpk
cmake --build build --config Debug --target GraphNavigatorCity
.\build\Debug\GraphNavigatorCity.exe maps\demo_district.gnpk
```

Реальный район: выгрузите .osm (JOSM/Overpass) и прогоните через `mapbake.py`. Пакеты из папки `maps` показываются на стартовом экране.
Модули: `mapbake/` (OSM → .gnpk), `client/data` (чтение с проверкой границ), `client/routing` (A*/Dijkstra/TSP на CSR + рабочий поток), `client/render` (здания, дороги, зебры, деревья, кварталы с отсечением), `client/ui` (виджеты, панель маршрута).
Управление: ЛКМ — двигать карту, короткий клик — точка старта/финиша, Shift+клик — промежуточная, ПКМ — вращать/наклон, колесо — масштаб, WASD/QE.

## Регрессии

`tests/package_tests.cpp`: усечённый и испорченный пакет, огромные счётчики, лишние байты, A*/Dijkstra/TSP на CSR, рабочий поток, сверка A* и Dijkstra на demo-пакете.


`tests/graph_tests.cpp`: кратчайший путь, параллельные рёбра, недостижимая/неизвестная цель, несколько стартов, повторяющиеся точки TSP, компактный XML, XML entities, порядок way/node, повторная загрузка, пустая и повреждённая карта.
