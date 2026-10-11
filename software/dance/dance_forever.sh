#!/bin/sh
# CRABORA -- dance on a loop until interrupted.
#
# Stand the robot up first; this does not do that for you:
#
#     python3 stand/stand.py
#     ./dance/dance_forever.sh
#
# Any arguments are passed straight through to dance.py, so:
#
#     ./dance/dance_forever.sh --amp 10 --speed 1000
#     ./dance/dance_forever.sh --moves twist shimmy
#
# Ctrl-C stops the loop. The running dance.py gets the same signal and
# returns the robot to its standing pose before exiting, so the robot
# does NOT go limp and drop -- torque is deliberately left on. Release
# it yourself once it is supported:
#
#     python3 utils/go_limp.py <id>
#
# The loop also stops on its own if dance.py fails, rather than
# hammering a robot that is already unhappy.

set -u

DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PYTHON=${PYTHON:-python3}

cycles=0
start=$(date +%s)

stop() {
    elapsed=$(( $(date +%s) - start ))
    printf '\n\nStopped after %d cycle(s) in %dm %ds.\n' \
        "$cycles" "$((elapsed / 60))" "$((elapsed % 60))"
    printf 'Torque is still ON and the robot should be standing.\n'
    exit 0
}
trap stop INT TERM

printf 'CRABORA dance loop -- Ctrl-C to stop\n'
printf 'Robot must already be standing (stand/stand.py).\n\n'

while :; do
    cycles=$((cycles + 1))
    printf '===== cycle %d =====\n' "$cycles"

    "$PYTHON" "$DIR/dance.py" --no-pause "$@"
    status=$?

    # 130 is dance.py's own Ctrl-C exit: it has already parked the robot
    # at the standing pose, so just stop quietly.
    if [ "$status" -eq 130 ]; then
        stop
    fi
    if [ "$status" -ne 0 ]; then
        printf '\ndance.py exited %d -- stopping the loop.\n' "$status"
        printf 'The robot is holding whatever pose it reached.\n'
        exit "$status"
    fi
done
