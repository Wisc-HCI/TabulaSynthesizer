# Tabula Synthesizer

This repository contains the task planner for Tabula. You can run the test cases without ROS, but the ROS setup is necessary if you want to use the accompnaying frontend UI (located in TabulaMobileUI repo).

## Installation

Installation involves a few extra steps if you're using ROS and not just running the test cases.

### Prerequisites

- Ubuntu 20.04 (If you are not using ROS, you can likely use another OS, as long as the python3 and java programs run!)
- An installation of python3 and java
- python3 libraries: nltk, spacy,contractions
- ROS Noetic (optional) 
- ROS TCP Endpoint (if using ROS)

#### Clone repo
Clone this repo in your ROS workspace. Assuming that your workspace is `~/catkin_ws` that would be:
```
cd ~/catkin_ws/src/
git clone git@github.com:dporfirio/TabulaSynthesizer.git 
```

If you are using ROS, don't forget you'll need to build the packages and source:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### Install Standford corenlp
```
cd TabulaSynthesizer
mkdir bin && cd bin
wget https://nlp.stanford.edu/software/stanford-corenlp-4.5.4.zip
unzip stanford-corenlp-4.5.4.zip
```

To run it, you'll need Java, which can be installed like: `sudo apt install openjdk-11-jre-headless`

#### Install Dependencies
You can try:
```
roscd synthesizer/src
python3 -m pip -r requirements.txt
```

If that doesn't work, you can install them manually:
```
python3 -m pip install nltk spacy contractions
python3 -m spacy download en_core_web_sm
```

Note, if you are having trouble with the spacy download you may need to install click first: `pip3 install click --upgrade`

#### Download necessary nltk resources
Use the python3 interpreter, like this. If they are not up-to-date already, they will download:
```
$ python3
Python 3.8.10 (default, Nov 14 2022, 12:59:47) 
[GCC 9.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import nltk
>>> nltk.download('wordnet')
[nltk_data] Downloading package wordnet to /home/<usr>/nltk_data...
[nltk_data]   Package wordnet is already up-to-date!
True
>>> nltk.download('verbnet3')
[nltk_data] Downloading package verbnet3 to /home/<usr>/nltk_data...
[nltk_data]   Package verbnet3 is already up-to-date!
True
>>> nltk.download('omw-1.4')
[nltk_data] Downloading package omw-1.4 to /home/<usr>/nltk_data...
[nltk_data]   Package omw-1.4 is already up-to-date!
True
>>> nltk.download('punkt')                                                                                              
[nltk_data] Downloading package punkt to /home/stegner/nltk_data...                                                     
[nltk_data]   Unzipping tokenizers/punkt.zip.                                                                           
True
>>> exit()
```


#### ROS setup (if using ROS)
Assuming that your workspace is `~/catkin_ws` and your IP address is `\<your_ip\>`, do the following:
```
cd ~/catkin_ws
echo "ROS_IP: \<your_ip\>" > src/TabulaSynthesizer/synthesizer/config/params.yaml
```

You will also need to install the ros_tcp_endpoint package. To do so:
1. Clone the package repo: https://github.com/Unity-Technologies/ROS-TCP-Endpoint to your `~/catkin_ws/src` folder
2. In the package, find this file: `ROS-TCP-Endpoint/src/ros_tcp_endpoint/default_server_endpoint.py`
3. In that file, change the shebang (first line) from `#!/usr/bin/env python` to `#!/usr/bin/env python3`
4. Build and source the package: 
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Running

# Stanford CoreNLP (required for both ROS and Non-ROS)

Before running, the Stanford CoreNLP server must be started. Make sure that this server is installed in `TabulaSynthesizer/bin` and make sure that you run it from where it's installed, otherwise the code will have runtime issues. --> in `bin` then you need to run it in the standford nlp folder it makes, not directly from the bin folder. For example:

```
cd ~/catkin_ws/src/TabulaSynthesizer/bin/stanford-corenlp-4.5.4/
java -mx4g -cp "*" edu.stanford.nlp.pipeline.StanfordCoreNLPServer -port 9000 -timeout 15000
```

# ROS (for use with frontend UI)

To run the ROS backend, do the following:

- Ensure the Stanford CoreNLP server is running in the correct location (see above).
- Also don't forget to configure the ROS master: 
```
export ROS_IP=\<your_ip\>
export ROS_MASTER_URI=http://\<your_ip\>:11311
```
- Then, start the synthesizer by running:

```
roslaunch synthesizer ctrl.launch
```

# Non-ROS (running the test cases without the UI)

config: Available actions of the robot are available to view at `synthesizer/src/action_primitives.json`. These would have to be configured based on the capabilities of the robot being used. The robot also knows certain entites, which are available to view at `synthesizer/src/entities.json`.

inputs: The test case inputs are located in `synthesizer/test_cases/` where each JSON file includes the world (labeled regions with any relevant objects nested in each region), natural language (nl - a list of speech utterances) and trajectories (a list of lists of waypoints). nl and trajectories are linked -- the first nl corresponds to the speech that accompanies the first list of waypoints in the trajectory. One pair of nl + list of waypoints is the equivalent of a recording that could be provided through the UI.

outputs: Outputs of the syntehsizer are generated in `synthesizer/src/test_files/temp`. We created an oracle file for each test case input located in `synthesizer/src/test_files/oracle`, which is used to check that the output of the syntehsizer is correct. The format of the /temp/ and /oracle/ files are the same. They are text files that list the steps for each provided recording.

To run the test cases, do the following:

1. Ensure the Stanford CoreNLP server is running in the correct location.

2. In another terminal windows, assuming you are in the `TabulaSynthesizer` folder, run:

```
cd synthesizer/src
./test.bash
```
