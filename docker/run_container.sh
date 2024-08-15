#!/bin/bash

xhost +

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker run \
    -it --rm \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/dev:/dev:rw" \
    --volume="`pwd`/checkpoints:/ros2_ws/install/pcdet_ros2/share/pcdet_ros2/checkpoints" \
    --name="ros2_torch_container" \
    --runtime=nvidia \
    --network=host \
    --ipc=host \
    --pid=host \
    --env UID=$(id -u) \
    --env GID=$(id -g) \
    ros2_torch:latest 
