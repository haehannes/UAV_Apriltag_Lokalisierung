#!/usr/bin/env bash
# control_console.sh
#
# Startet die uXRCE-Bedienkonsole (curses) DIREKT im aktuellen Terminal.
# Kein ros2 launch, kein extra xterm-Fenster.
#
# Warum kein Launch-File? ros2 launch / ExecuteProcess leiten stdin/stdout
# des Prozesses um. Ein curses-UI braucht aber ein echtes interaktives TTY,
# sonst kommen die Tastendruecke nicht an. Deshalb wird hier direkt gestartet.
#
# Parameter haben Defaults (siehe unten) und lassen sich auf zwei Wegen
# ueberschreiben:
#   1) per Umgebungsvariable:   demo_speed_mps=0.2 ./control_console.sh
#   2) per zusaetzlichem -p:     ./control_console.sh -p demo_speed_mps:=0.2
#
# Aktuelle Defaults:
#   demo_edge_length_m=2.0   demo_speed_mps=0.3   descent_speed_mps=0.3
#   takeoff_offset_m=1.5     demo_center_x=0.0    demo_center_y=0.0
# >>> Bei Bedarf anpassen <

# Repo-Root finden: vom Skript-Verzeichnis aufwaerts, bis der Ordner
# "Apriltag_UAV_Lokalisierung" erreicht ist. Damit sind die Pfade
# unabhaengig vom Aufrufort und vom konkreten Speicherort des Wrappers.
# Repo-Root direkt ueber das Home-Verzeichnis. Simpel, aber setzt voraus,
# dass das Repo unter ~/Apriltag_UAV_Lokalisierung liegt.
REPO_ROOT="${HOME}/Apriltag_UAV_Lokalisierung"

# Pfade ausgehend vom Repo-Root.
SCRIPT="${REPO_ROOT}/workspaces/ws_humble/src/control_package/control/control_console_uxrce.py"
WORKSPACE_SETUP="${REPO_ROOT}/workspaces/ws_humble/install/setup.bash"

# Pfade ausgehend vom Repo-Root.
SCRIPT="${REPO_ROOT}/workspaces/ws_humble/src/control_package/control/control_console_uxrce.py"
WORKSPACE_SETUP="${REPO_ROOT}/workspaces/ws_humble/install/setup.bash"

# Default-Parameter (per Umgebungsvariable ueberschreibbar).
demo_edge_length_m="${demo_edge_length_m:-2.0}"
demo_speed_mps="${demo_speed_mps:-0.3}"
descent_speed_mps="${descent_speed_mps:-0.3}"
takeoff_offset_m="${takeoff_offset_m:-1.5}"
demo_center_x="${demo_center_x:-0.0}"
demo_center_y="${demo_center_y:-0.0}"

if [ ! -f "${SCRIPT}" ]; then
  echo "FEHLER: Skript nicht gefunden: ${SCRIPT}" >&2
  exit 1
fi

# Workspace sourcen, falls vorhanden (liefert rclpy, px4_msgs, ...).
if [ -f "${WORKSPACE_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP}"
fi

# exec -> die Konsole uebernimmt das aktuelle Terminal direkt (TTY fuer curses).
# "$@" am Ende: zusaetzliche --ros-args/-p ueberschreiben die Defaults oben.
exec python3 "${SCRIPT}" --ros-args \
  -p demo_edge_length_m:="${demo_edge_length_m}" \
  -p demo_speed_mps:="${demo_speed_mps}" \
  -p descent_speed_mps:="${descent_speed_mps}" \
  -p takeoff_offset_m:="${takeoff_offset_m}" \
  -p demo_center_x:="${demo_center_x}" \
  -p demo_center_y:="${demo_center_y}" \
  "$@"