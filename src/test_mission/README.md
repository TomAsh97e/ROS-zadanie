# test_mission

Misja testowa dla swiata `SImWorld`.

Sekwencja:
- start i wznoszenie,
- lot do tylu nad `barrel_center` (`x=-30, y=0`),
- zrzut piłki do beczki bez oczekiwania na `ball_grabber`.

Uwaga:
- misja nie wymaga działania `ball_grabber` do startu,
- przy zrzucie piłka jest ustawiana pod dronem i pozwalana na opadanie do beczki.

Potwierdzone z mapy PX4:
- dron startuje domyslnie w `(0, 0, 0)`,
- plansze z pilkami sa z przodu w okolicach `x=5`,
- beczki sa za dronem w okolicach `x=-30`,
- srodkowa beczka to `barrel_center` na `(-30, 0, 0.5)`.

Uruchomienie:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ludek/projects/Simulation_last_mile_logistics
source install/setup.bash
ros2 launch test_mission barrel_drop_test.launch.py
```
