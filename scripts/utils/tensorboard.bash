#!/usr/bin/env bash
### Start a TensorBoard server with the default logdir relative to the repository
### Usage: tensorboard.bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "$(dirname "${SCRIPT_DIR}")")"

## If desired, use the last edited log directory
USE_LAST="${USE_LAST:-"false"}"
if [ "${USE_LAST}" == "true" ]; then
    LOGDIR="$(find "${LOGDIR}" -type d -exec ls -dt {} + | head -n 1)"
fi

## Start TensorBoard
TENSORBOARD_CMD=(
    "${ISAAC_SIM_PYTHON:-"python3"}" "${ISAAC_SIM_PATH:-"/root/isaac-sim"}/tensorboard"
    --logdir "${LOGDIR:-"${REPOSITORY_DIR}/logs"}"
    --bind_all
)
echo -e "\033[1;90m[TRACE] ${TENSORBOARD_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${TENSORBOARD_CMD[*]}
