# Zadanie rekrutacyjne: profilowanie ROS i przepisanie na C++

Wasze zadanie to zmierzyć, które części naszego systemu misji obciążają komputer najbardziej, przepisać jeden lub dwa z tych fragmentów z Pythona na C++ i pokazać liczbami, co to dało.

## Zadanie

Zadanie ma cztery etapy. Najpierw mierzycie, dopiero potem przepisujecie.

1. **Pomiar bazowy.** Ustalcie jeden powtarzalny scenariusz, np. pełny przelot misji od startu do pobrania pierwszej próbki (świat HydroLab jest za każdym razem taki sam), i zmierzcie na nim obecny system:
    - zużycie CPU i pamięci przez każdy węzeł osobno,
    - rzeczywistą częstotliwość kluczowych topiców,
    - czas wykonania najważniejszych callbacków,
    - opóźnienie od klatki kamery do opublikowania wyniku detekcji.
2. **Analiza.** Na podstawie pomiarów i flamegraphów wskażcie, które węzły i funkcje kosztują najwięcej. Zapiszcie też, ile całego obciążenia to nasze węzły, a ile Gazebo i PX4.
3. **Przepisanie na C++.** Wybierzcie jeden lub dwa węzły, na których zysk będzie największy, i przepiszcie je jako nowy pakiet `ament_cmake` z `rclcpp`. Wybór uzasadnijcie pomiarami z etapu 2.
4. **Porównanie.** Podmieńcie węzły w launchu na wersje C++, powtórzcie dokładnie ten sam pomiar co w etapie 1 i zestawcie wyniki przed i po.

Na koniec oddajecie link do swojego forka repo, w którym jest:

- [ ] pakiet C++ z przepisanymi węzłami, który się buduje i działa w misji,
- [ ] launch, w którym da się wybrać wersję Python albo C++,
- [ ] krótki raport (Markdown): scenariusz pomiarowy, wyniki przed i po, flamegraphy i uzasadnienie wyboru węzłów.

Instrukcja instalacji i uruchomienia symulacji jest w [README.md](README.md).

## System

Dron w symulacji HydroLab startuje z lądowiska, leci do kolorowych basenów, ustawia się nad nimi i pobiera próbki wody. Całość działa na ROS 2 Jazzy, PX4 SITL i Gazebo Harmonic. Pakiet misji `hydrolab_mission` to osiem węzłów w Pythonie, które startują razem z `launch/system_launch.py`.

| Węzeł | Plik | Co robi | Jak często pracuje |
| --- | --- | --- | --- |
| `ball_detector` | `ball_detector_node.py` | Bierze obraz z kamery w Gazebo i szuka obiektów po kolorze (OpenCV) | Każda klatka kamery, gdy detekcja jest włączona; wynik co 0,2 s |
| `ball_grabber` | `ball_grabber_node.py` | Śledzi pozycje drona i obiektów w Gazebo, „przyczepia” obiekt do drona | Każda wiadomość z pozycjami z Gazebo + timer 20 Hz |
| `raycasted_approach_coordinator_node` | `raycasted_approach_coordinator_node.py` | Koryguje cel podejścia na podstawie czujnika odległości i orientacji gimbala | Timer 50 Hz + odometria i czujnik odległości z PX4 |
| `pilot` | `pilot_node.py` | Zamienia cele misji na komendy dla PX4 | Timer 10 Hz + odometria z PX4 |
| `hover_controller` | `hover_controller_node.py` | Utrzymuje drona nad basenem na podstawie detekcji | 10 Hz |
| `gimbal_controller` | `gimbal_controller_node.py` | Steruje gimbalem kamery: przeszukiwanie i stabilizacja | 10 Hz |
| `gimbal_tracker` | `gimbal_tracker_node.py` | Prowadzi gimbal za wykrytym celem | 10 Hz |
| `behavior_tree` | `mission_behavior_tree_node.py` | Logika misji jako drzewo zachowań (py_trees) | 5 Hz |

Parametry wszystkich węzłów są w `runtime_config/hydrolab_mission/mission_params.yaml` i `camera_config.yaml`. Węzły wymieniają się częścią danych przez `std_msgs/String` z JSON-em w środku, a z Gazebo rozmawiają bezpośrednio przez gz-transport.

## Wymagania wobec wersji C++

Nowy węzeł ma być bezpośrednią podmianą starego. Reszta systemu nie powinna zauważyć różnicy.

- Te same nazwy topiców, typy wiadomości i ten sam format JSON-a w środku.
- Te same parametry, czytane z `mission_params.yaml` pod tą samą nazwą węzła.
- To samo zachowanie: misja z węzłem C++ ma przechodzić tak samo jak z pythonowym.
- Pakiet buduje się przez `colcon build` bez ręcznych kroków, a zależności są wpisane w `package.xml` i `CMakeLists.txt`.
- Do Gazebo używacie biblioteki gz-transport w C++, do obrazu OpenCV w C++.
- Pythonowe węzły zostają w repo. Między wersjami ma się dać przełączać w launchu.

## Narzędzia

Nie musicie użyć wszystkich. Ważne, żeby pomiary przed i po były zrobione tak samo.

| Narzędzie | Do czego |
| --- | --- |
| `htop` / `top`, `pidstat` | CPU i pamięć każdego procesu w czasie |
| `ros2 topic hz`, `ros2 topic bw`, `ros2 topic delay` | Częstotliwość, przepustowość i opóźnienie topiców |
| [py-spy](https://github.com/benfred/py-spy) (`top`, `record`) | Profil i flamegraph węzłów w Pythonie bez zmiany kodu |
| `perf` + flamegraph | Profil węzłów w C++ |
| [ros2_tracing](https://github.com/ros2/ros2_tracing) | Czas wykonania callbacków i opóźnienia w ROS 2 |
| Własne znaczniki czasu w kodzie | Opóźnienie od klatki do detekcji, czas jednego callbacku |
