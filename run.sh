#!/bin/bash

cd bin/stanford-corenlp-4.5.0/
java -mx4g -cp "*" edu.stanford.nlp.pipeline.StanfordCoreNLPServer -port 9000 -timeout 15000 &
roslaunch synthesizer ctrl.launch
