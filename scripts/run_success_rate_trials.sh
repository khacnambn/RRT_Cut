#!/usr/bin/env bash

# Success rate of the map/planner currently selected in user_config.yaml, for
# dynamic (pedestrian) scenes only.
#
#   success rate = trials that reach the goal without touching a pedestrian
#                  ---------------------------------------------------------
#                            trials that produced a verdict
#
# Timeouts are discarded completely: they leave the denominator untouched and
# the script runs another attempt in their place, until TRIAL_COUNT verdicts
# exist or MAX_ATTEMPTS is reached.  Startup failures (see the known Gazebo
# spawn timeout) and crashes of the helper nodes are discarded the same way,
# because none of them says anything about the planner.
#
# A verdict is one of
#   SUCCESS    goal reached, no pedestrian closer than the collision threshold
#   COLLISION  a pedestrian came closer than the threshold (goal reached or not)
#   ABORTED    move_base gave up on finding a valid plan or control
#
# This script never edits user_config.yaml, goal_config.yaml or planner YAMLs,
# and never touches planner or local-planner sources.
#
# Usage:
#   ./run_success_rate_trials.sh [valid_trial_count]      # default 20
#
# Environment overrides:
#   COLLISION_DIST_M=0.5     centre distance counted as a collision
#   NAVIGATION_TIMEOUT_S=180 per-trial navigation budget
#   STARTUP_TIMEOUT_S=90     per-trial Gazebo/move_base startup budget
#   MAX_ATTEMPTS=<n>         hard cap on attempts (default 3 x trial count)
#   STOP_ON_COLLISION=1      end the trial as soon as a collision happens
#                            (with 0, a trial that collides and then runs out
#                            of time is still discarded as a timeout; its
#                            collision count stays in trials.csv)
#   ABORT_AS_FAILURE=1       count move_base aborts as failures (0 discards them)
#   KEEP_TRACES=0            1 also copies the distance trace of clean trials
#   PURGE_SIM_STATE=1        wipe Gazebo/RViz/ROS state between trials (0 keeps it)

set -u

export LC_ALL=C

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scripts_dir="${workspace_dir}/scripts"
user_config="${workspace_dir}/src/user_config/user_config.yaml"
monitor_py="${scripts_dir}/pedestrian_collision_monitor.py"
purge_sh="${scripts_dir}/purge_sim_state.sh"

trial_count="${1:-20}"
startup_timeout="${STARTUP_TIMEOUT_S:-90}"
navigation_timeout="${NAVIGATION_TIMEOUT_S:-180}"
# turtlebot3_waffle footprint circumscribes a 0.2570 m radius; a walking person
# is about 0.25 m wide at the shoulders, so 0.5 m between centres is contact.
collision_dist="${COLLISION_DIST_M:-0.5}"
max_attempts="${MAX_ATTEMPTS:-$((trial_count * 3))}"
stop_on_collision="${STOP_ON_COLLISION:-1}"
abort_as_failure="${ABORT_AS_FAILURE:-1}"
keep_traces="${KEEP_TRACES:-0}"
purge_sim_state="${PURGE_SIM_STATE:-1}"

if ! [[ "${trial_count}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [positive_trial_count]" >&2
  exit 2
fi

map_name="$(awk -F'"' '/^map:[[:space:]]*"/ { print $2; exit }' "${user_config}")"
world_name="$(awk -F'"' '/^world:[[:space:]]*"/ { print $2; exit }' "${user_config}")"
scene_name="${world_name:-${map_name}}"
planner_name="$(awk -F'"' '/robot1_global_planner:[[:space:]]*"/ { print $2; exit }' "${user_config}")"
robot_type="$(awk -F'"' '/robot1_type:[[:space:]]*"/ { print $2; exit }' "${user_config}")"

if [[ -z "${map_name}" || -z "${planner_name}" ]]; then
  echo "Cannot read map or robot1_global_planner from ${user_config}" >&2
  exit 2
fi

if [[ ! -x "${monitor_py}" && ! -f "${monitor_py}" ]]; then
  echo "Missing collision monitor: ${monitor_py}" >&2
  exit 2
fi

# This metric only means something in a scene with pedestrians, so refuse a
# static world instead of silently reporting a 100% success rate.
world_file="${workspace_dir}/src/sim_env/worlds/${world_name}.world"
if [[ -z "${world_name}" || ! -f "${world_file}" ]]; then
  echo "Cannot find the world file for world \"${world_name}\" (${world_file})" >&2
  exit 2
fi
actor_list="$(grep -o '<actor name="[^"]*"' "${world_file}" | sed 's/.*name="//; s/"$//' | paste -sd, -)"
if [[ -z "${actor_list}" ]]; then
  echo "World \"${world_name}\" has no <actor>: this is a static scene." >&2
  echo "Select a *_with_pedestrians world in ${user_config}, or use run_benchmark_trials.sh." >&2
  exit 2
fi

case "${planner_name}" in
  rrt|rrt_star|rrt_astar|rrt_astar_node_limit|rrt_astar_confidence|rrt_cut|informed_rrt|informed_rrt_star|quick_informed_rrt|rrt_connect)
    plan_topic="/move_base/SamplePlanner/plan"
    ;;
  a_star|jps|gbfs|dijkstra|d_star|lpa_star|voronoi|d_star_lite|theta_star|lazy_theta_star|s_theta_star|hybrid_a_star)
    plan_topic="/move_base/GraphPlanner/plan"
    ;;
  aco|pso|ga)
    plan_topic="/move_base/EvolutionaryPlanner/plan"
    ;;
  *)
    echo "Unsupported global planner for automatic plan-topic selection: ${planner_name}" >&2
    exit 2
    ;;
esac

run_tag="$(date +%Y%m%d_%H%M%S)"
result_dir="${workspace_dir}/benchmark_results/${scene_name}_${planner_name}_successrate_${run_tag}"
mkdir -p "${result_dir}"

source /opt/ros/noetic/setup.bash
source "${workspace_dir}/devel/setup.bash"

if timeout 2 rosnode list >/dev/null 2>&1; then
  echo "A ROS master is already running. Stop it before starting this benchmark." >&2
  exit 3
fi

results_file="${result_dir}/results.txt"
csv_file="${result_dir}/trials.csv"

{
  printf '%s - SUCCESS RATE (canh dong)\n\n' "$(printf '%s' "${planner_name}" | tr '[:lower:]' '[:upper:]')"
  printf 'scene                %s\n' "${scene_name}"
  printf 'map                  %s\n' "${map_name}"
  printf 'planner              %s\n' "${planner_name}"
  printf 'robot                %s\n' "${robot_type}"
  printf 'pedestrians          %s\n' "${actor_list}"
  printf 'collision threshold  %.4f m (robot 0.2570 m + nguoi ~0.25 m)\n' "${collision_dist}"
  printf 'navigation timeout   %s s (timeout bi loai hoan toan)\n' "${navigation_timeout}"
  printf 'target valid trials  %s (toi da %s lan chay)\n' "${trial_count}" "${max_attempts}"
  printf 'move_base abort      %s\n\n' "$([[ "${abort_as_failure}" == "1" ]] && echo "tinh la FAIL" || echo "loai bo")"
  printf '%-5s %-8s %-22s %12s %8s %10s\n' "lan" "attempt" "ket qua" "min_dist(m)" "collide" "t(s)"
} > "${results_file}"

echo "attempt,trial,status,counted,min_dist_m,collision_events,duration_s,arm_time_s,first_collision_actor" > "${csv_file}"

main_pid=""
trace_pid=""
monitor_pid=""

stop_group() {
  local leader="${1:-}"
  [[ -n "${leader}" ]] || return 0

  if kill -0 -- "-${leader}" 2>/dev/null; then
    kill -INT -- "-${leader}" 2>/dev/null || true
    sleep 3
  fi
  if kill -0 -- "-${leader}" 2>/dev/null; then
    kill -TERM -- "-${leader}" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 -- "-${leader}" 2>/dev/null; then
    kill -KILL -- "-${leader}" 2>/dev/null || true
  fi
  wait "${leader}" 2>/dev/null || true
}

tagged_pids() {
  pgrep -f "__log:=/tmp/ros_motion_planning_${run_tag}_" 2>/dev/null || true
}

signal_tagged_processes() {
  local signal="$1"
  local pids=()
  mapfile -t pids < <(tagged_pids)
  if [[ "${#pids[@]}" -gt 0 ]]; then
    kill "-${signal}" "${pids[@]}" 2>/dev/null || true
  fi
}

wait_for_clean_shutdown() {
  local attempt
  for attempt in $(seq 1 20); do
    if [[ -z "$(tagged_pids)" ]] && ! timeout 1 rosnode list >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

cleanup_run() {
  # The monitor first: it must flush its summary before the master goes away.
  stop_group "${monitor_pid}"
  stop_group "${trace_pid}"
  stop_group "${main_pid}"

  signal_tagged_processes INT
  sleep 3
  signal_tagged_processes TERM
  sleep 2
  signal_tagged_processes KILL

  monitor_pid=""
  trace_pid=""
  main_pid=""

  if ! wait_for_clean_shutdown; then
    echo "Cleanup failed: ROS/Gazebo processes from run ${run_tag} are still alive." >&2
    return 1
  fi
}

purge_state() {
  # Called only after cleanup_run reported a clean shutdown, so nothing is
  # reading the state that gets deleted here.  The current run tag is spared:
  # its /tmp directories hold this run's own logs.
  [[ "${purge_sim_state}" == "1" ]] || return 0
  [[ -f "${purge_sh}" ]] || return 0
  bash "${purge_sh}" --keep "${run_tag}" --quiet || true
}

on_exit() {
  local code=$?
  trap - EXIT INT TERM
  cleanup_run
  exit "${code}"
}
trap on_exit EXIT INT TERM

summary_field() {
  # summary_field <file> <key> [fallback]
  local value=""
  if [[ -f "$1" ]]; then
    value="$(awk -F': ' -v key="$2" '$1 == key { print $2; exit }' "$1")"
  fi
  if [[ -z "${value}" || "${value}" == "nan" ]]; then
    printf '%s' "${3:-}"
  else
    printf '%s' "${value}"
  fi
}

echo "Scene: ${scene_name} (map ${map_name})"
echo "Planner: ${planner_name}"
echo "Pedestrians: ${actor_list}"
echo "Collision threshold: ${collision_dist} m"
echo "Valid trials wanted: ${trial_count} (max ${max_attempts} attempts)"
echo "Results: ${result_dir}"

# Start from a clean slate: Gazebo, RViz and roslaunch state from earlier runs
# (including other sessions) goes before the first trial.
purge_state

counted=0
success_count=0
collision_count=0
aborted_count=0
discarded_timeout=0
discarded_startup=0
discarded_infra=0
discarded_aborted=0
attempt=0

while (( counted < trial_count && attempt < max_attempts )); do
  attempt=$((attempt + 1))
  attempt_id="$(printf '%02d' "${attempt}")"
  export ROS_LOG_DIR="/tmp/ros_motion_planning_${run_tag}_${attempt_id}"
  mkdir -p "${ROS_LOG_DIR}"
  main_log="${ROS_LOG_DIR}/main.log"
  trace_log="${ROS_LOG_DIR}/path_trace.log"
  monitor_log="${ROS_LOG_DIR}/collision_monitor.log"
  monitor_summary="${ROS_LOG_DIR}/collision_summary.yaml"
  monitor_trace="${ROS_LOG_DIR}/distances.csv"
  yaml_file="${ROS_LOG_DIR}/result.yaml"

  echo "[attempt ${attempt_id}] starting (valid ${counted}/${trial_count})"
  setsid stdbuf -oL -eL bash -c "cd '${scripts_dir}' && exec ./main.sh" >"${main_log}" 2>&1 &
  main_pid=$!

  ready=0
  for _ in $(seq 1 "${startup_timeout}"); do
    if grep -q "SpawnModel: Successfully spawned entity" "${main_log}" 2>/dev/null &&
       rosparam get /move_base/base_global_planner >/dev/null 2>&1; then
      ready=1
      break
    fi
    if grep -q "SpawnModel: Failure" "${main_log}" 2>/dev/null || ! kill -0 "${main_pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done

  if [[ "${ready}" -ne 1 ]]; then
    discarded_startup=$((discarded_startup + 1))
    echo "[attempt ${attempt_id}] DISCARDED startup_failed"
    printf '%-5s %-8s %-22s %12s %8s %10s\n' "-" "${attempt}" "DISCARDED startup" "-" "-" "-" >> "${results_file}"
    printf '%d,,startup_failed,0,,,,,\n' "${attempt}" >> "${csv_file}"
    cp "${main_log}" "${result_dir}/discarded_attempt_${attempt_id}_main.log" 2>/dev/null || true
    if ! cleanup_run; then
      echo "Benchmark stopped to prevent the next attempt from using stale ROS/TF state." >&2
      exit 4
    fi
    purge_state
    sleep 3
    continue
  fi

  # Let AMCL, TF and move_base subscriptions settle before path_trace_node
  # publishes its one-shot goal from goal_config.yaml.
  sleep 5

  setsid stdbuf -oL -eL python3 "${monitor_py}" \
    --actors "${actor_list}" \
    --robot-model "${robot_type}" \
    --threshold "${collision_dist}" \
    --output "${monitor_summary}" \
    --trace "${monitor_trace}" >"${monitor_log}" 2>&1 &
  monitor_pid=$!

  setsid stdbuf -oL -eL rosrun path_trace_node path_trace_node \
    _plan_topic:="${plan_topic}" _output_file:="${yaml_file}" >"${trace_log}" 2>&1 &
  trace_pid=$!

  started_at="$(date +%s)"
  status="timeout"
  for _ in $(seq 1 "${navigation_timeout}"); do
    if [[ "${stop_on_collision}" == "1" ]] && grep -q "^COLLISION " "${monitor_log}" 2>/dev/null; then
      status="collision"
      break
    fi
    if [[ -s "${yaml_file}" ]] && grep -q "Goal reached!" "${trace_log}" 2>/dev/null; then
      status="goal_reached"
      break
    fi
    # move_base aborts with either "valid control" (controller gave up) or
    # "valid plan" (planner gave up).
    if grep -qE "Aborting because a valid (control|plan) could not be found" "${main_log}" 2>/dev/null; then
      status="navigation_aborted"
      break
    fi
    if ! kill -0 "${trace_pid}" 2>/dev/null; then
      status="trace_exited"
      break
    fi
    if ! kill -0 "${monitor_pid}" 2>/dev/null; then
      status="monitor_exited"
      break
    fi
    sleep 1
  done
  duration=$(( $(date +%s) - started_at ))

  # Stop the monitor first so its final summary is on disk before it is read.
  stop_group "${monitor_pid}"
  monitor_pid=""
  collisions="$(summary_field "${monitor_summary}" "collision_events" "0")"
  min_dist="$(summary_field "${monitor_summary}" "min_distance_m" "")"
  arm_time="$(summary_field "${monitor_summary}" "arm_time_s" "")"
  hit_actor="$(summary_field "${monitor_summary}" "first_collision_actor" "")"

  if [[ "${status}" == "goal_reached" ]]; then
    if [[ "${collisions}" =~ ^[0-9]+$ ]] && (( collisions > 0 )); then
      status="collision"
    else
      status="success"
    fi
  fi

  verdict=""
  case "${status}" in
    success)
      verdict="SUCCESS"
      counted=$((counted + 1))
      success_count=$((success_count + 1))
      ;;
    collision)
      verdict="COLLISION ${hit_actor}"
      counted=$((counted + 1))
      collision_count=$((collision_count + 1))
      ;;
    navigation_aborted)
      if [[ "${abort_as_failure}" == "1" ]]; then
        verdict="ABORTED"
        counted=$((counted + 1))
        aborted_count=$((aborted_count + 1))
      else
        verdict="DISCARDED aborted"
        discarded_aborted=$((discarded_aborted + 1))
      fi
      ;;
    timeout)
      verdict="DISCARDED timeout"
      discarded_timeout=$((discarded_timeout + 1))
      ;;
    *)
      verdict="DISCARDED ${status}"
      discarded_infra=$((discarded_infra + 1))
      ;;
  esac

  if [[ "${verdict}" == DISCARDED* ]]; then
    trial_label="-"
    counted_flag=0
  else
    trial_label="${counted}"
    counted_flag=1
  fi

  printf '%-5s %-8s %-22s %12s %8s %10s\n' \
    "${trial_label}" "${attempt}" "${verdict}" "${min_dist:--}" "${collisions:-0}" "${duration}" \
    >> "${results_file}"
  printf '%d,%s,%s,%d,%s,%s,%d,%s,%s\n' \
    "${attempt}" "${trial_label}" "${status}" "${counted_flag}" \
    "${min_dist}" "${collisions}" "${duration}" "${arm_time}" "${hit_actor}" >> "${csv_file}"

  # Keep the evidence for everything that is not a clean success.
  if [[ "${status}" != "success" || "${keep_traces}" == "1" ]]; then
    cp "${monitor_summary}" "${result_dir}/attempt_${attempt_id}_${status}_collision_summary.yaml" 2>/dev/null || true
    cp "${monitor_trace}" "${result_dir}/attempt_${attempt_id}_${status}_distances.csv" 2>/dev/null || true
  fi
  if [[ "${status}" != "success" ]]; then
    cp "${main_log}" "${result_dir}/attempt_${attempt_id}_${status}_main.log" 2>/dev/null || true
    cp "${trace_log}" "${result_dir}/attempt_${attempt_id}_${status}_path_trace.log" 2>/dev/null || true
    cp "${monitor_log}" "${result_dir}/attempt_${attempt_id}_${status}_monitor.log" 2>/dev/null || true
  fi

  echo "[attempt ${attempt_id}] ${verdict} (min_dist ${min_dist:--} m, ${duration} s)"
  if ! cleanup_run; then
    echo "Benchmark stopped to prevent the next attempt from using stale ROS/TF state." >&2
    exit 4
  fi
  purge_state
  sleep 3
done

discarded_total=$((discarded_timeout + discarded_startup + discarded_infra + discarded_aborted))

if (( counted > 0 )); then
  rate="$(awk -v s="${success_count}" -v n="${counted}" 'BEGIN { printf "%.4f", s / n }')"
  rate_pct="$(awk -v s="${success_count}" -v n="${counted}" 'BEGIN { printf "%.2f", 100 * s / n }')"
else
  rate="nan"
  rate_pct="nan"
fi

{
  printf '\n=== TONG KET ===\n'
  printf 'attempts                     %d\n' "${attempt}"
  printf 'trial duoc tinh (denominator) %d\n' "${counted}"
  printf '  SUCCESS  (den dich, khong va cham)  %d\n' "${success_count}"
  printf '  COLLISION (va cham nguoi di bo)     %d\n' "${collision_count}"
  printf '  ABORTED  (move_base bo cuoc)        %d\n' "${aborted_count}"
  printf 'bi loai (khong vao mau)       %d\n' "${discarded_total}"
  printf '  timeout                             %d\n' "${discarded_timeout}"
  printf '  startup_failed                      %d\n' "${discarded_startup}"
  printf '  aborted (ABORT_AS_FAILURE=0)        %d\n' "${discarded_aborted}"
  printf '  node crash / khac                   %d\n' "${discarded_infra}"
  printf '\nSUCCESS RATE = %d / %d = %s (%s %%)\n' "${success_count}" "${counted}" "${rate}" "${rate_pct}"
} >> "${results_file}"

if (( counted < trial_count )); then
  printf 'CANH BAO: chi thu duoc %d/%d trial hop le trong %d attempts.\n' \
    "${counted}" "${trial_count}" "${attempt}" >> "${results_file}"
fi
if (( discarded_timeout > 0 )); then
  printf 'LUU Y: %d lan timeout da bi loai hoan toan; bao cao kem con so nay.\n' \
    "${discarded_timeout}" >> "${results_file}"
fi

echo
sed -n '/=== TONG KET ===/,$p' "${results_file}"
echo
echo "Results: ${results_file}"
echo "Per-trial CSV: ${csv_file}"

trap - EXIT INT TERM
