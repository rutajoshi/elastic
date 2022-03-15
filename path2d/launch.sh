#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch simulation first
./simviz_path2d &
SIMVIZ_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $SIMVIZ_PID
}

sleep 2

# launch controller
./controller_path2d &
CONTROLLER_PID=$!

sleep 1

# wait for simviz to quit
wait $SIMVIZ_PID

# onnce simviz dies, kill controller
kill $CONTROLLER_PID
