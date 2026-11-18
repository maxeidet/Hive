#!/usr/bin/env bash
# Build the swarm backend inside the running px4-gazebo-vnc container and run it.
# Start the container and drone 0 first (docker/start-dev.sh or start-stellar.sh).
#
# Usage:
#   ./mount-backend.sh                 interactive swarm shell (same as `shell`)
#   ./mount-backend.sh shell [N]       shell using the first N drones of config/swarm.yaml
#   ./mount-backend.sh spawn [N]       start PX4 drones 1..N-1 at their swarm.yaml poses
#   ./mount-backend.sh spawn TYPE      start ONE more drone of TYPE (see below) at the next free id
#   ./mount-backend.sh single [args]   Milestone 1 test (tests/single_drone_test.py --help)
#   ./mount-backend.sh multi [args]    Milestone 2 test (tests/multi_drone_swarm_test.py --help)
#   ./mount-backend.sh intercept [args] chase test: drone 2 chases drone 0 at 50 m using mock
#                                      radar fixes (tests/intercept_test.py --help, e.g.
#                                      --pattern figure8 --speed 5 --csv /tmp/run.csv)
#   ./mount-backend.sh scenario NAME [--sim] [args]  run a scenario (swarm_backend/scenarios/):
#                                      in Gazebo, or with --sim in the fast simulation (no
#                                      Gazebo or PX4 needed, seconds per run). NAME: attack,
#                                      baseline_eval, intercept_baseline; `scenario --help`
#                                      lists them, `scenario NAME --help` their arguments.
#                                      attack and baseline_eval use scenarios/surveillance.yaml
#                                      unless SWARM_CONFIG is set. Without the container:
#                                      cd ros2_ws/src/swarm_backend && python3 -m swarm_backend.scenarios NAME --sim
#   ./mount-backend.sh baseline-intercept [args]  = scenario intercept_baseline: one intercept
#                                      baseline scenario of the report (e.g. --aspect 90 --speed 8 --seed 3)
#   ./mount-backend.sh fakedetect [args] fake camera detector: publishes /px4_2/detection/target_position
#                                      and target_offset from Gazebo's true positions (attack: in
#                                      swarm.yaml; --realistic for delay, noise and blind zone)
#   ./mount-backend.sh attack [args]   baseline attack: drone 2 takes off, searches and flies into
#                                      what the detection topics report (pure pursuit), lands on
#                                      Ctrl-C (swarm_backend/nodes/attack_node.py --help)
#   ./mount-backend.sh bird spawn|move|wander|remove [args]  birds in the running world
#                                      (swarm_backend/birds.py --help), e.g. bird spawn dove
#                                      --at 20 0 30, then bird wander bird_1 --around 20 0 30
#   ./mount-backend.sh boxtest [args]  try Gazebo's bounding-box camera for training labels: a bare
#                                      camera, a labelled x500 and dove at 3-150 m, images with the
#                                      boxes drawn in data/box_test/ (swarm_backend/box_test.py --help)
#   ./mount-backend.sh gendata --run NAME [--count N] [args]  labelled YOLO training images
#                                      in data/gen/NAME/ (+ NAME/labels_check.png): random camera, sun and drones in the
#                                      running world (hive_base); --yaml --val R --test R writes
#                                      data/gen/data.yaml, split by run (swarm_backend/gen_data.py)
#   ./mount-backend.sh genlive ID --run NAME [--count N] [args]  the same, but drone ID (spawn
#                                      gimbal_labels) flies and its own gimbal camera takes the
#                                      images (swarm_backend/gen_live.py)
#   ./mount-backend.sh makecrops [args] cut-outs around the baseline's candidates in data/gen/,
#                                      for YOLO as a second stage, in data/crops/ (same split by
#                                      run; swarm_backend/make_crops.py)
#   ./mount-backend.sh showlabels DIR [--n 5] [--out F]  draw the YOLO labels of a run (e.g.
#                                      data/gen/b05) with zoomed cut-outs, to check them
#                                      by eye; writes DIR/labels_check.png (training/show_labels.py)
#   ./mount-backend.sh comparedet DIR [--weights F]  baseline vs YOLO on a labelled run (e.g.
#                                      data/gen/b14): recall by size and distance, false alarms,
#                                      ms per frame (training/compare_detectors.py)
#   ./mount-backend.sh fitrange [--fit R..] [--test R..]  distance from box size: the fixed 0.5 m rule
#                                      vs sizes fitted on data/gen/ (fit b06-b13, test b14 b15):
#                                      error by size and distance, plots in data/results/range_fit/
#                                      (training/fit_range.py; offline, changes no live code)
#   ./mount-backend.sh yololabels DIR  YOLO label files (labels/*.txt) for the images snap saved in
#                                      DIR, from the boxes in their .json (swarm_backend/yolo_labels.py)
#   ./mount-backend.sh attacktest [args] = scenario attack: defenders 1-3 patrol the arc,
#                                      the base station sends one at hostile drone 4 flying in
#                                      toward the base, real camera detectors (started by the
#                                      scenario). A hit removes the hostile from Gazebo and the
#                                      defender flies back to its slot; one CSV row per run in
#                                      data/surveillance_attack_runs.csv (e.g. --bearing -30 0 30
#                                      --runs 3, --fake-detector --realistic, --sim)
#   ./mount-backend.sh baseline-eval intercept|detect|summary [args]  = scenario baseline_eval:
#                                      the report's 30-seed scenarios flown by the 3
#                                      defenders on the arc with the base station, real
#                                      detectors and pure pursuit, scored with the report's
#                                      metrics into data/baseline_eval/ (resumable; summary
#                                      prints the tables; e.g. intercept --aspects 0 --speeds 4
#                                      --seeds 0-4; --sim: data/baseline_eval_sim/)
#   ./mount-backend.sh basestation     surveillance base station: reads the defenders' sightings and
#                                      status and sends orders (nodes/base_station_node.py,
#                                      surveillance: in swarm.yaml). Use scenarios/surveillance.yaml
#   ./mount-backend.sh patrol ID [args] one defender on the arc: flies to its slot, sweeps its camera,
#                                      attacks when told to (nodes/patrol_node.py). One per
#                                      defender, each with its own detector (`fakedetect --id ID`)
#   ./mount-backend.sh ui [args]       operator UI on http://localhost:8090: where the drones are,
#                                      where their gimbals look, and the search area (published
#                                      on /base/search_area). Read-only towards the drones
#                                      (swarm_backend/nodes/operator_ui.py --help)
#   ./mount-backend.sh mapsnap [args]  one picture of the running world from straight above, as
#                                      the operator UI's background map (map_snapshot.py --help,
#                                      default 800 m across, --half-size 600 for more). Restart `ui`.
#   ./mount-backend.sh demo [N]        formation demo (swarm_node)
#   ./mount-backend.sh legacy          old backend_node + keyboard teleop
#   ./mount-backend.sh view [ID]       show drone ID in RViz with its cameras and lidars, default 1.
#                                      Any drone type; one without sensors shows where it is.
#                                      (`lidar [ID]` and `camera [ID]` do the same)
#   ./mount-backend.sh map [ID] [args]  build a 3D map (OctoMap) from drone ID's depth camera or
#                                      lidar (default 0) and show it in RViz: /mapping/octomap_full.
#                                      Args: --ns NS --resolution M --max-range M (mapping.py --help)
#   ./mount-backend.sh maptest [args]  3D mapping flight test: drone 0 (depth or lidar model) maps,
#                                      flies 6 m north and checks that the map grew
#                                      (tests/mapping_test.py --help, e.g. --distance 10 --bearing 90)
#   ./mount-backend.sh explore [X,Y RADIUS] [ARG:=VALUE ...]  3D mapping with the DAEP planner:
#                                      drone 0 (depth or lidar model) flies to X,Y (Gazebo, ENU),
#                                      explores the box X,Y +- RADIUS, saves ros2_ws/data/map.bt
#                                      and lands when done. Without X,Y: the box in DAEP's
#                                      px4_exploration.yaml around the start point. Needs the
#                                      private DAEP forks in ros2_ws/src, built once with
#                                      colcon build --packages-up-to daep (launch/
#                                      daep_exploration.launch.py, e.g. explore 13.5,-14 7 alt:=4 z_max:=5)
#   ./mount-backend.sh scanmap [args]  draw the scanned 3D map from above as the operator UI's
#                                      background map (ui/static/map.png + map.json). Default
#                                      data/map.bt; --watch 10 redraws the running map every 10 s
#                                      while `map` or `explore` flies (octomap_snapshot.py --help)
#   ./mount-backend.sh snap [ID] [args] save drone ID's camera images as PNGs in data/Manualimages/ of
#                                      the repo, default drone 2, one per second until Ctrl-C
#                                      (e.g. snap 2 --count 10 --label sky_30m; --help for more).
#                                      Each image gets a .json with the true positions of the
#                                      drones and the distance and direction to them (Gazebo)
#   ./mount-backend.sh predict [METHOD] [args]  run a detector on the saved images in
#                                      data/Manualimages/ and put the images with boxes and a
#                                      results.csv in data/results/METHOD/. METHOD: baseline
#                                      (default) or yolo (not trained yet). Other folders:
#                                      --images /workspace/data/X --out /workspace/data/Y
#   ./mount-backend.sh detect [ID] [args] camera detector on drone ID (default 2): publishes
#                                      /px4_ID/detection/target_offset, and a debug
#                                      image on /px4_ID/detection/debug (swarm_backend/detect/live.py
#                                      --help, e.g. --detector baseline --save /workspace/detect_frames)
#   ./mount-backend.sh detecttest [args] baseline detection flight test: drone 2 watches drone 0
#                                      fly away from it (tests/detection_test.py --help)
#   ./mount-backend.sh restart [N] [-w WORLD] [-m MODEL] [--headless|--gui]  restart Gazebo and the drones without
#                                      restarting the container: stops every test, shell, RViz,
#                                      PX4 and Gazebo, starts Gazebo with drone 0 again (log:
#                                      /tmp/px4_0.log), then drones 1..N-1 of swarm.yaml if N is
#                                      given. Keeps the world, drone 0 model and headless setting
#                                      it was running with, else baylands and gz_x500.
#                                      Example: ./mount-backend.sh restart && scenarios/scenario1.sh
#
# spawn TYPE: plain, lidar (360° 3D, our model), lidar2d, lidar_front, lidar_down (1-beam
#   rangefinders), camera, camera_down, depth (RGB + depth), flow, gimbal, gimbal_labels
#   (gimbal + bounding-box cameras for training labels, our model), or any PX4_SIM_MODEL
#   such as gz_x500_depth. It takes the lowest instance id that is not running and that id's
#   swarm.yaml spawn_pose (so the shell can fly it), else a free spot at (4*id, -5, 0).
#   The shell only flies drones listed in swarm.yaml.
#   Example: ./mount-backend.sh spawn camera && ./mount-backend.sh view 1
#   A gimbal drone's camera is pointed from the shell: gimbal(id, pitch, yaw)
# spawn [N]: drones use the `model:` from swarm.yaml, else $MODEL (default gz_x500).
# Custom models in worlds/models/ (e.g. gz_x500_lidar_3d) are copied into the container first.
# RViz opens on the GUI display: the screen with start-stellar.sh --native-gui, the VNC
# desktop with start-dev.sh or --vnc. It cannot open with --headless.
# Every command reads the swarm.yaml in this folder, so edits apply without rebuilding.
# SWARM_CONFIG=/workspace/... (a path inside the container) uses another config, as the
# scripts in scenarios/ do.
set -e

CONTAINER=px4-gazebo-vnc
PKG=/workspace/ros2_ws/src/swarm_backend
CONFIG="${SWARM_CONFIG:-$PKG/config/swarm.yaml}"
MODEL="${MODEL:-gz_x500}"

SETUP="
  source /opt/ros/jazzy/setup.bash
  source /opt/px4_ros2/install/local_setup.bash
  cd /workspace/ros2_ws
  colcon build --symlink-install --packages-select swarm_backend >/dev/null || colcon build --symlink-install --packages-select swarm_backend
  source install/setup.bash
"

case "${1:-}" in
  -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
esac

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "Container $CONTAINER is not running. Start it with docker/start-dev.sh (or start-stellar.sh) first."
  exit 1
fi

run() {
  docker exec -it "$CONTAINER" bash -c "$SETUP
  $1"
}

num_arg() {
  if [ -n "$1" ]; then echo "--num-drones $1"; fi
}

copy_models() {
  docker exec "$CONTAINER" bash -c "cp -r /workspace/worlds/models/. /opt/px4-gazebo/share/gz/models/"
}

start_drone() {
  local id="$1" x="$2" y="$3" z="$4" model="$5"
  echo "Starting drone $id ($model) at Gazebo ($x, $y, $z), log: /tmp/px4_$id.log in the container"
  # -dit: detached, but with a terminal so the PX4 shell stays alive.
  # Models without their own PX4 airframe (our custom ones) fly with the airframe number in
  # worlds/models/<model>/airframe if there is one, else the plain x500 airframe.
  docker exec -dit "$CONTAINER" bash -c "
    export PX4_SIM_MODEL=$model PX4_GZ_MODEL_POSE='$x,$y,$z,0,0,0'
    ls /opt/px4-gazebo/etc/init.d-posix/airframes/*_$model >/dev/null 2>&1 || export PX4_SYS_AUTOSTART=\$(cat /workspace/worlds/models/${model#gz_}/airframe 2>/dev/null || echo 4001)
    /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i $id > /tmp/px4_$id.log 2>&1"
}

model_for() {
  # Short names for `spawn TYPE`.
  case "$1" in
    plain)       echo gz_x500 ;;
    lidar)       echo gz_x500_lidar_3d ;;
    lidar2d)     echo gz_x500_lidar_2d ;;
    lidar_front) echo gz_x500_lidar_front ;;
    lidar_down)  echo gz_x500_lidar_down ;;
    camera)      echo gz_x500_mono_cam ;;
    camera_down) echo gz_x500_mono_cam_down ;;
    depth)       echo gz_x500_depth ;;
    flow)        echo gz_x500_flow ;;
    gimbal)      echo gz_x500_gimbal ;;
    gimbal_labels) echo gz_x500_gimbal_labels ;;
    gz_*)        echo "$1" ;;
    *)           return 1 ;;
  esac
}

spawn_one() {
  local model id pose
  if ! model=$(model_for "$1"); then
    echo "Unknown drone type: $1 (see --help)"
    exit 1
  fi
  # Lowest instance id >= 1 that no PX4 in the container is using.
  local running
  running=$(docker exec "$CONTAINER" ps -eo args | grep -oE 'px4(-gazebo)? -i [0-9]+' | awk '{print $NF}' | sort -un)
  id=1
  while grep -qx "$id" <<< "$running"; do id=$((id + 1)); done
  pose=$(docker exec "$CONTAINER" python3 -c "
import yaml
poses = {d['id']: d.get('spawn_pose', [0, 0, 0]) for d in yaml.safe_load(open('$CONFIG'))['drones']}
print(*poses.get($id, [4 * $id, -5, 0]))
")
  copy_models
  start_drone "$id" $pose "$model"
  echo "Give it ~15 s to appear in Gazebo. Then: ./mount-backend.sh view $id"
}

spawn() {
  # "id x y z model" for every drone but 0 (which the start script already runs).
  local drones
  drones=$(docker exec "$CONTAINER" python3 -c "
import yaml
drones = yaml.safe_load(open('$CONFIG'))['drones'][:${1:-1000}]
for d in drones[1:]:
    print(d['id'], *d.get('spawn_pose', [0, 0, 0]), d.get('model', '$MODEL'))
")
  if [ -z "$drones" ]; then
    echo "Nothing to spawn: swarm.yaml (or N) has only drone 0."
    return
  fi
  copy_models
  while read -r id x y z model; do
    if docker exec "$CONTAINER" pgrep -f "px4(-gazebo)? -i $id( |\$)" >/dev/null; then
      echo "Drone $id is already running."
      continue
    fi
    start_drone "$id" "$x" "$y" "$z" "$model"
  done <<< "$drones"
  echo "Give them ~15 s to appear in Gazebo and connect; status() in the shell shows when they are ready."
}

view() {
  # Bridge the drone's Gazebo camera/lidar topics and poses to ROS 2 and open RViz (sensor_view.py).
  run "python3 -m swarm_backend.sensor_view --id ${1:-1} --rviz $PKG/config/sensors.rviz"
}

mapping() {
  # Build a 3D map (OctoMap) from the drone's depth camera or lidar (mapping.py).
  local id="${1:-0}"
  [ $# -gt 0 ] && shift
  run "python3 -m swarm_backend.mapping --id $id --rviz $PKG/config/sensors.rviz $*"
}

explore() {
  # DAEP exploration (launch/daep_exploration.launch.py), from ros2_ws so data/ lands there.
  local args=""
  if [[ "${1:-}" =~ ^-?[0-9.]+,-?[0-9.]+$ ]]; then
    args="center:=$1 radius:=${2:-6}"
    shift; [ $# -gt 0 ] && shift
  fi
  run "ros2 pkg prefix daep >/dev/null 2>&1 || { echo 'DAEP is not built. Clone the daep and daep_msgs forks into ros2_ws/src, then: colcon build --packages-up-to daep'; exit 1; }
  ros2 launch swarm_backend daep_exploration.launch.py $args $*"
}

restart() {
  local n="" world="" model="" timeout="${WORLD_TIMEOUT:-180}" settings want_headless=""
  while [ $# -gt 0 ]; do
    case "$1" in
      -w|--world) world="$2"; shift 2 ;;
      -m|--model) model="$2"; shift 2 ;;
      --headless) want_headless=1; shift ;;
      --gui)      want_headless=0; shift ;;
      [0-9]*)     n="$1"; shift ;;
      *) echo "Unknown restart argument: $1 (see --help)"; exit 1 ;;
    esac
  done
  # The world, model and HEADLESS that drone 0 runs with now, else those of the last restart
  # (kept in /tmp/hive_sim.env in the container). The [x] in each pattern stops pgrep/pkill
  # from matching the bash -c running them.
  settings=$(docker exec "$CONTAINER" bash -c "
    gz=\$(pgrep -af '[g]z sim.* -s ' | grep -oE '[^/ ]+\.sdf' | head -1)
    p0=\$(pgrep -f '[b]in/px4 -i 0\$' | head -1)
    if [ -n \"\$p0\" ]; then
      tr '\0' '\n' < /proc/\$p0/environ | grep -E '^(PX4_SIM_MODEL|HEADLESS)='
      [ -n \"\$gz\" ] && echo PX4_GZ_WORLD=\${gz%.sdf}
    elif [ -f /tmp/hive_sim.env ]; then
      cat /tmp/hive_sim.env
    fi; true")
  world="${world:-$(sed -n 's/^PX4_GZ_WORLD=//p' <<< "$settings")}"
  model="${model:-$(sed -n 's/^PX4_SIM_MODEL=//p' <<< "$settings")}"
  world="${world:-baylands}" model="${model:-gz_x500}"
  local headless
  headless=$(grep -E '^HEADLESS=' <<< "$settings" || true)
  # --headless: no Gazebo GUI window (saves a lot of CPU, e.g. for gendata); --gui: back again
  [ "$want_headless" = 1 ] && headless=HEADLESS=1
  [ "$want_headless" = 0 ] && headless=""

  echo "Stopping tests, shells, RViz, PX4 and Gazebo (the container and MicroXRCE agent keep running)..."
  # Killed ROS/PX4 processes leave Fast DDS shared memory behind, and new drones then never
  # reach ROS. `fastdds shm clean` removes it.
  docker exec "$CONTAINER" bash -c "
    pkill -f '[t]ests/[a-z_]*\.py|[s]warm_shell|-m [s]warm_backend\.'
    pkill -f '[r]viz2'; pkill -f '[p]arameter_bridge'
    pkill -f '[p]x4(-gazebo)? -i [0-9]'; pkill -f '[g]z sim'
    for _ in \$(seq 10); do pgrep -f '[g]z sim|[b]in/px4 -i' >/dev/null || break; sleep 1; done
    pkill -9 -f '[g]z sim'; pkill -9 -f '[b]in/px4 -i'
    source /opt/ros/jazzy/setup.bash; ros2 daemon stop >/dev/null 2>&1; fastdds shm clean >/dev/null 2>&1
    printf '%s\n' PX4_GZ_WORLD=$world PX4_SIM_MODEL=$model $headless > /tmp/hive_sim.env
    true"

  echo "Starting Gazebo ($world) and drone 0 ($model), log: /tmp/px4_0.log in the container"
  docker exec "$CONTAINER" bash -c "cp /workspace/worlds/*.sdf /opt/px4-gazebo/share/gz/worlds/ 2>/dev/null || true"
  copy_models
  # Same as the start scripts, but detached: the PX4 shell is not on a terminal.
  docker exec -dit "$CONTAINER" bash -c "
    export $headless PX4_GZ_WORLD_TIMEOUT=$timeout PX4_GZ_WORLD=$world PX4_SIM_MODEL=$model PX4_GZ_MODEL_POSE=0,0,0,0,0,0
    /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0 > /tmp/px4_0.log 2>&1"

  echo "Waiting for the Gazebo world..."
  for _ in $(seq "$timeout"); do
    if docker exec "$CONTAINER" gz topic -l 2>/dev/null | grep -q "^/world/$world/clock\$"; then break; fi
    sleep 1
  done
  if ! docker exec "$CONTAINER" gz topic -l 2>/dev/null | grep -q "^/world/$world/clock\$"; then
    echo "Gazebo did not come up. See /tmp/px4_0.log in the container."
    exit 1
  fi
  if [ -n "$n" ]; then spawn "$n"; fi

  # ros2 topic list still shows the old drones' topics, so ask each PX4 if it is connected.
  echo "Waiting for the drones to connect to ROS (can take a minute or two)..."
  local ids status
  ids=$(docker exec "$CONTAINER" ps -eo args | grep -oE 'px4(-gazebo)? -i [0-9]+' | awk '{print $NF}' | sort -un)
  for _ in $(seq "$timeout"); do
    status=$(docker exec "$CONTAINER" bash -c "for i in $(echo $ids); do /opt/px4-gazebo/bin/px4-uxrce_dds_client --instance \$i status 2>/dev/null | head -1 | grep -q ', connected' || echo \$i; done")
    [ -z "$status" ] && break
    sleep 1
  done
  if [ -n "$status" ]; then
    echo "Not connected to ROS yet: drone(s) $(echo $status). Logs: /tmp/px4_<id>.log in the container."
    exit 1
  fi
  echo "Gazebo and drones $(echo $ids) are up and talking to ROS."
}

detect() {
  local id=2
  if [[ "$1" =~ ^[0-9]+$ ]]; then id="$1"; shift; fi
  run "python3 -m swarm_backend.detect.live --id $id $*"
}

cmd="${1:-shell}"
shift || true
case "$cmd" in
  shell)  run "ros2 run swarm_backend swarm_shell --config $CONFIG $(num_arg "$1")" ;;
  spawn)
    if [ -z "$1" ] || [[ "$1" =~ ^[0-9]+$ ]]; then spawn "$1"; else spawn_one "$1"; fi ;;
  single) run "python3 $PKG/tests/single_drone_test.py --config $CONFIG $*" ;;
  multi)  run "python3 $PKG/tests/multi_drone_swarm_test.py --config $CONFIG $*" ;;
  intercept) run "python3 $PKG/tests/intercept_test.py --config $CONFIG $*" ;;
  detecttest) run "python3 $PKG/tests/detection_test.py --config $CONFIG $*" ;;
  scenario)
    # --config only when SWARM_CONFIG is set: each scenario has its own default config.
    run "python3 -m swarm_backend.scenarios $* ${SWARM_CONFIG:+--config $SWARM_CONFIG}" ;;
  baseline-intercept) run "python3 -m swarm_backend.scenarios intercept_baseline --config $CONFIG $*" ;;
  fakedetect) run "python3 -m swarm_backend.nodes.fake_detection --config $CONFIG $*" ;;
  attack) run "python3 -m swarm_backend.nodes.attack_node --config $CONFIG $*" ;;
  hivemind) run "python3 -m swarm_backend.nodes.hivemind_node --config $CONFIG $*" ;;
  bird)   run "python3 -m swarm_backend.birds $*" ;;
  boxtest) run "python3 -m swarm_backend.box_test $*" ;;
  gendata)
    # After a run, draw its labels to DIR/labels_check.png to check them by eye.
    if run "python3 -m swarm_backend.gen_data $*"; then
      gen_run="" gen_out=/workspace/data/gen
      while [ $# -gt 0 ]; do
        case "$1" in --run) gen_run="$2"; shift ;; --out) gen_out="$2"; shift ;; --yaml) gen_run=""; break ;; esac
        shift
      done
      if [ -n "$gen_run" ]; then
        run "cd /workspace && python3 training/show_labels.py $gen_out/$gen_run --n 10" || true
      fi
    fi ;;
  genlive) run "python3 -m swarm_backend.gen_live $*" ;;
  makecrops) run "python3 -m swarm_backend.make_crops $*" ;;
  yololabels) run "cd /workspace && python3 -m swarm_backend.yolo_labels $*" ;;
  showlabels) run "cd /workspace && python3 training/show_labels.py $*" ;;
  fitrange) run "cd /workspace && python3 training/fit_range.py $*" ;;
  comparedet) run "cd /workspace && python3 training/compare_detectors.py $*" ;;
  basestation) run "python3 -m swarm_backend.nodes.base_station_node --config $CONFIG $*" ;;
  patrol) run "python3 -m swarm_backend.nodes.patrol_node --config $CONFIG --id $*" ;;
  baseline-eval) run "python3 -m swarm_backend.scenarios baseline_eval --config ${SWARM_CONFIG:-$PKG/scenarios/surveillance.yaml} $*" ;;
  attacktest) run "python3 -m swarm_backend.scenarios attack --config ${SWARM_CONFIG:-$PKG/scenarios/surveillance.yaml} $*" ;;
  ui)     run "python3 -m swarm_backend.nodes.operator_ui --config $CONFIG $*" ;;
  mapsnap) run "python3 -m swarm_backend.map_snapshot $*" ;;
  demo)   run "ros2 run swarm_backend swarm_node --ros-args -p config_file:=$CONFIG -p num_drones:=${1:-0}" ;;
  legacy) run "ros2 run swarm_backend backend_node & ros2 run swarm_backend keyboard" ;;
  view|lidar|camera) view "$1" ;;
  map) mapping "$@" ;;
  maptest) run "python3 $PKG/tests/mapping_test.py --config $CONFIG $*" ;;
  scanmap) run "python3 -m swarm_backend.octomap_snapshot $*" ;;
  explore) explore "$@" ;;
  predict) run "python3 -m swarm_backend.detect.predict $*" ;;
  snap)
    id=2
    if [[ "${1:-}" =~ ^[0-9]+$ ]]; then id="$1"; shift; fi
    run "python3 -m swarm_backend.save_images --id $id $*" ;;
  detect) detect "$@" ;;
  restart) restart "$@" ;;
  *) echo "Unknown command: $cmd (see --help)"; exit 1 ;;
esac
