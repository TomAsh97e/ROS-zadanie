# Simulation_Hydrolab

Symulacja misji HydroLab: dron (PX4, model `x500_gimbal`) startuje z lądowiska, leci do kolorowych basenów, ustawia się nad nimi i pobiera próbki wody. Logika misji to pakiet ROS 2 w Pythonie.

> **Zadanie rekrutacyjne:** opis tego, co jest do zrobienia, jest w [ZADANIE.md](ZADANIE.md). Ten plik opisuje, jak postawić i uruchomić symulację.

## Struktura repo

| Ścieżka | Zawartość |
| --- | --- |
| `src/hydrolab_mission` | Główny pakiet misji: węzły `pilot`, `raycasted_approach_coordinator_node`, `hover_controller`, `gimbal_controller`, `gimbal_tracker`, `ball_detector`, `ball_grabber`, `behavior_tree` |
| `src/px4_msgs` | Submoduł z wiadomościami PX4 dla ROS 2 |
| `src/test_mission` | Misja testowa z projektu last mile logistics (zakłada świat z beczkami, w tym świecie nie zadziała) |
| `runtime_config/hydrolab_mission` | Parametry misji i kamery używane przy uruchamianiu |
| `SimWorld_hydrolab` | Świat Gazebo z basenami, generator świata i skrypt startowy |

Launch czyta parametry z `runtime_config/hydrolab_mission`, więc zmiana YAML-i nie wymaga przebudowy. Plik `src/hydrolab_mission/config/mission_params.yaml` to starsza kopia i nie jest używany przez launch.

## Wymagania

- Ubuntu 24.04 (natywnie albo VM; na Windowsie nie zadziała)
- ROS 2 Jazzy
- PX4-Autopilot z gałęzi `main` (submoduł `px4_msgs` odpowiada commitowi PX4 `d8d3bee`, kwiecień 2026). Przy innej wersji PX4 przestaw submoduł na pasującą wersję `px4_msgs`.
- Gazebo Harmonic (instaluje je skrypt PX4 `Tools/setup/ubuntu.sh`)
- Micro XRCE-DDS Agent (`MicroXRCEAgent`)
- Python: `python3-opencv`, `python3-numpy` oraz bindingi Gazebo `python3-gz-transport13` i `python3-gz-msgs10`
- Opcjonalnie QGroundControl jako `~/QGroundControl.AppImage`

Biblioteka `py_trees` jest dołączona do pakietu misji, nie trzeba jej instalować.

## Instalacja

1. Sklonuj repo razem z submodułem:

    ```bash
    git clone --recurse-submodules https://github.com/TomAsh97e/ROS-zadanie.git
    ```

    Jeśli repo jest już sklonowane bez submodułu:

    ```bash
    git submodule update --init
    ```

2. Zainstaluj PX4 i Gazebo zgodnie z [dokumentacją PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html):

    ```bash
    git clone --recursive https://github.com/PX4/PX4-Autopilot.git
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    ```

3. Zainstaluj Micro XRCE-DDS Agent zgodnie z [instrukcją PX4 dla ROS 2](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client).

4. Zbuduj workspace (z katalogu głównego repo):

    ```bash
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    ```

## Konfiguracja skryptu startowego

Skrypt `SimWorld_hydrolab/launch_envhydrolab.sh` ma wpisane ścieżki z komputera autora. Przed pierwszym uruchomieniem zmień w nim:

- `PX4_ROOT` na katalog PX4-Autopilot,
- `HYDROLAB_CONFIG_DIR` na `<repo>/runtime_config/hydrolab_mission`,
- linię `source .../install/setup.bash` na `<repo>/install/setup.bash`.

Generator świata `build_world.py` zapisuje `hydrolab_built.sdf` obok siebie i do katalogu światów PX4. Ten katalog wskaż zmienną środowiskową:

```bash
export PX4_WORLDS_DIR=<PX4-Autopilot>/Tools/simulation/gz/worlds
```

Świat jest generowany deterministycznie, więc każde uruchomienie daje ten sam układ basenów.

## Uruchomienie

Skrypt startowy uruchamia wszystko naraz: generuje świat, odpala PX4 SITL z Gazebo, agenta DDS, opcjonalnie QGC, a na końcu launch misji. Trzeba go odpalić z katalogu `SimWorld_hydrolab`:

```bash
cd SimWorld_hydrolab
./launch_envhydrolab.sh
```

Żeby uruchamiać misję osobno (np. przy pomiarach albo podmianie węzłów), zakomentuj w skrypcie linię `ros2 launch hydrolab_mission system_launch.py &` i w drugim terminalu uruchom:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export HYDROLAB_CONFIG_DIR=$(pwd)/runtime_config/hydrolab_mission
ros2 launch hydrolab_mission system_launch.py
```

Zamiast zmiennej `HYDROLAB_CONFIG_DIR` można podać `config_dir:=<ścieżka>` jako argument launcha.

## Zamykanie

`Ctrl+C` w terminalu ze skryptem startowym zamyka całą symulację. Gdy coś zostanie w tle:

```bash
killall -9 px4 gz ruby MicroXRCEAgent
```
