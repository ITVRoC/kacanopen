#!/bin/sh

SCRIPTDIR=$(dirname "$(readlink -f "$0")")
grep -rl "static const bool debug = true;" "$SCRIPTDIR/../core/include" "$SCRIPTDIR/../master/include" "$SCRIPTDIR/../ros_bridge/include" | xargs sed -i 's/static const bool debug = true;/static const bool debug = false;/g'
