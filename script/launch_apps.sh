#!/bin/bash

CATKIN_WS=${PWD}/../../

# launch roscore
launch_roscore () {
    count=`ps -aef|grep roscore|grep -v grep|wc -l`
    if [ 0 == $count ];then
        roscore &
        sleep 20
        rosparam set /rosout/omit_topics true
        ROSCOREPIDS=`ps -aef|grep ros/melodic|grep -v grep|awk '{print $2}'`
        if [ -z "$ROSCOREPIDS" ]; then
            errorExit "roscore is not running."
        fi
        echo =============== Launch roscore ! ==============
    fi
}

# launch rviz
launch_rviz () {
    PIDS=`ps -aef|grep rviz|grep -v grep|awk '{print $2}'`
    for pid in $PIDS
    do
        kill -9 $pid
    done

    cd ${CATKIN_WS}
    source ./devel/setup.bash
    roslaunch test_suite visualization.launch &

    sleep 2
    echo =============== Launch Rviz ! ==============
}

# launch visualization
launch_visualization () {
    PIDS=`ps -aef|grep visual|grep -v grep|awk '{print $2}'`
    for pid in $PIDS
    do
        kill -9 $pid
    done

    cd ${CATKIN_WS}/devel/lib/test_suite
    ./visual &
    sleep 2
    VISPIDS=`ps -aef|grep visual|grep -v grep|awk '{print $2}'`
    if [ -z "$VISPIDS" ]; then
        errorExit "visual is not running."
    fi  
    echo ================== launch visualization ! ====================
}

# launch localization
launch_localization () {
    LOCCFG="${CATKIN_WS}/devel/lib/localization/conf.yaml"
    PIDS=`ps -aef|grep localization|grep -v grep|awk '{print $2}'`
    for pid in $PIDS
    do
        kill -9 $pid
    done
    if [ ! -f "$LOCCFG" ]; then
        ln -s ${CATKIN_WS}/devel/lib/localization/conf.yaml $LOCCFG
    fi
    cd ${CATKIN_WS}/devel/lib/localization
    ./localization conf.yaml &
    sleep 2
    LOCPIDS=`ps -aef|grep localization|grep -v grep|awk '{print $2}'`
    if [ -z "$LOCPIDS" ]; then
        errorExit "localization is not running."
    fi      
    echo ================== launch localization ! ====================
}

launch_roscore
launch_rviz
launch_visualization
launch_localization