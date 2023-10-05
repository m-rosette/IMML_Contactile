docker run -it -d \
    --name contactile_sensor \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --device /dev/dri \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=".:/root/ros2_ws" \
    --net=host \
    --privileged \
    osrf/ros:humble-desktop-full    
    # imml_contactile-dev    


    
export containerId=$(docker ps -l -q)
