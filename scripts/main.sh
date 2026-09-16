#!/usr/bin/env bash
set -euo pipefail

# Locate the workspace from this script, so it can be started from any
# terminal directory. The selected configuration must live in src/user_config.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"
config_file="${1:-user_config.yaml}"
config_path="${workspace_dir}/src/user_config/${config_file}"

if [[ ! -f "${config_path}" ]]; then
    echo "Khong tim thay file cau hinh: ${config_path}" >&2
    exit 1
fi

source "${workspace_dir}/devel/setup.bash"
python3 "${workspace_dir}/src/plugins/dynamic_xml_config/main_generate.py" "${config_file}"
roslaunch sim_env main.launch
