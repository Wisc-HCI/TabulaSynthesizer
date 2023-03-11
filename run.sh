#!/bin/bash

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch synthesizer ctrl.launch &
cd bin/stanford-corenlp-4.5.0/
java -mx4g -cp "*" edu.stanford.nlp.pipeline.StanfordCoreNLPServer -port 9000 -timeout 15000 &

