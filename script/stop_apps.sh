#!/bin/bash

source /opt/ros/melodic/setup.bash

PIDS=`ps -aef|grep localization|grep -v grep|awk '{print $2}'`
for pid in $PIDS
do
    kill -9 $pid
done

PIDS=`ps -aef|grep visual|grep -v grep|awk '{print $2}'`
for pid in $PIDS
do
    kill -9 $pid
done

PIDS=`ps -aef|grep ros/melodic|grep -v grep|awk '{print $2}'`
for pid in $PIDS
do
    kill -9 $pid
done