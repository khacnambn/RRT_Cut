#!/usr/bin/env bash

# Delete the state Gazebo, RViz and roslaunch regenerate on every start, so the
# next run cannot inherit anything from the previous one.
#
# Removed:
#   ~/.gazebo/{client,server,gz}-<port>   per-port Gazebo state directories
#   ~/.gazebo/ogre.log                    renderer log
#   ~/.gazebo/{log,paging,rendering}/*    Gazebo logs and terrain/render caches
#   /tmp/gazebo-$USER-rtshaderlibcache    compiled Ogre shader cache
#   ~/.rviz/persistent_settings           RViz window/display state
#   ~/.ros/log/*                          every roslaunch/node log of past runs
#   ~/.ros/roscore-*.pid                  stale master pid files
#   /tmp/ros_motion_planning_*            benchmark log dirs of earlier runs
#
# Never touched: ~/.gazebo/models and ~/.gazebo/media (downloaded assets the
# worlds need), ~/.ros/rosdep, the rospack caches, and anything in the repo -
# the .rviz files under src/ are configuration, not state.
#
# Usage:
#   ./purge_sim_state.sh [--keep <tag>] [--dry-run] [--force] [--quiet]
#
#   --keep <tag>  spare /tmp/ros_motion_planning_* dirs whose name contains tag
#                 (the benchmark passes its own run tag to keep its evidence)
#   --dry-run     only list what would be deleted
#   --force       purge even while gzserver/gzclient/rviz/rosmaster are alive
#   --quiet       one summary line instead of the full list
#
# Environment:
#   PURGE_GUI_INI=1  also delete ~/.gazebo/gui.ini (resets the saved camera)

set -u
shopt -s nullglob

keep_tag=""
dry_run=0
force=0
quiet=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --keep) keep_tag="${2:-}"; shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    --force) force=1; shift ;;
    --quiet) quiet=1; shift ;;
    -h|--help) sed -n '3,30p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

home="${HOME:?HOME is not set}"
if [[ "${home}" == "/" ]]; then
  echo "Refusing to purge with HOME=/" >&2
  exit 2
fi

# Deleting state under a live simulator corrupts the run that is using it.
# A dry run only lists, so it is allowed while the simulation is up.
if [[ "${force}" -ne 1 && "${dry_run}" -ne 1 ]]; then
  for process in gzserver gzclient rviz rosmaster; do
    if pgrep -x "${process}" >/dev/null 2>&1; then
      echo "purge_sim_state: ${process} is still running; nothing deleted." >&2
      echo "Stop the simulation first, or pass --force." >&2
      exit 3
    fi
  done
fi

targets=()
add() {
  [[ -e "$1" || -L "$1" ]] && targets+=("$1")
  return 0
}

gazebo_home="${home}/.gazebo"
for path in "${gazebo_home}"/client-* "${gazebo_home}"/server-* "${gazebo_home}"/gz-*; do
  add "${path}"
done
add "${gazebo_home}/ogre.log"
for sub in log paging rendering; do
  for path in "${gazebo_home}/${sub}"/*; do
    add "${path}"
  done
done
if [[ "${PURGE_GUI_INI:-0}" == "1" ]]; then
  add "${gazebo_home}/gui.ini"
fi
add "/tmp/gazebo-${USER:-$(id -un)}-rtshaderlibcache"

add "${home}/.rviz/persistent_settings"
for path in "${home}"/.rviz/*.bak; do
  add "${path}"
done

for path in "${home}"/.ros/log/*; do
  add "${path}"
done
for path in "${home}"/.ros/roscore-*.pid; do
  add "${path}"
done

for path in /tmp/ros_motion_planning_*; do
  if [[ -n "${keep_tag}" && "${path}" == *"${keep_tag}"* ]]; then
    continue
  fi
  add "${path}"
done

if [[ "${#targets[@]}" -eq 0 ]]; then
  [[ "${quiet}" -eq 1 ]] || echo "purge_sim_state: nothing to delete."
  exit 0
fi

freed="$(du -shc --apparent-size -- "${targets[@]}" 2>/dev/null | tail -1 | cut -f1)"

if [[ "${dry_run}" -eq 1 ]]; then
  echo "purge_sim_state --dry-run: ${#targets[@]} paths, ${freed:-?} would be freed"
  printf '  %s\n' "${targets[@]}"
  exit 0
fi

if [[ "${quiet}" -ne 1 ]]; then
  printf '  rm %s\n' "${targets[@]}"
fi

rm -rf -- "${targets[@]}"
echo "purge_sim_state: deleted ${#targets[@]} paths (${freed:-?})${keep_tag:+, kept *${keep_tag}*}"
