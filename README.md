# Scout HRI Robot System: Thai Voice Command Recognition

A ROS 2-based human--robot interaction system for controlling a portable
Scout Robot using Thai voice commands. This repository contains
operator-side voice recognition and GUI software, along with robot-side
ROS 2 control and motor-driver software.

This README is based on the paper **"Development of a Reliable and
Responsive Thai Voice Command Recognition System for Portable Scout
Robot Control."** Installation commands below are practical setup
guidance; verify dependencies and hardware settings against the source
code on your branch.

## Research overview

The system recognizes five Thai commands:

  Command      Thai phrase   Intended action
  ------------ ------------- -----------------
  `FORWARD`    เดินหน้า        Move forward
  `BACKWARD`   ถอยหลัง        Move backward
  `LEFT`       เลี้ยวซ้าย       Turn left
  `RIGHT`      เลี้ยวขวา       Turn right
  `STOP`       หยุด           Stop

The paper describes Faster-Whisper with the Whisper Small model
configured for Thai, followed by post-ASR processing: Thai ASR error
correction, command-aware phonetic normalization, combined
lexical--phonetic similarity, command grammar constraints,
confidence/ambiguity rejection, and a STOP-specific recovery mechanism.

The architecture separates speech recognition from robot motion control.
The operator computer recognizes speech and sends a command over ROS 2;
a Raspberry Pi handles robot-side command relay, differential-drive
control, and motor communication.

### Reported results

  ------------------------------------------------------------------------------
  Metric                                      Baseline           Proposed system
  -------------------------- ------------------------- -------------------------
  Actual-command accuracy                          58%                       93%

  False Rejection Rate (FRR)                       42%                        7%

  STOP recall                                      40%                       95%

  False Acceptance Rate                            48%                       52%
  (FAR) for                                            
  non-command/near-command                             
  trials                                               

  Mean latency                                 4.487 s                   4.958 s

  P90 latency                                  4.761 s                   7.131 s
  ------------------------------------------------------------------------------

The evaluation used 100 intended-command trials (20 per command) and 100
non-command/near-command trials. Recognition of intended commands
improved, but false acceptance and latency also increased. This is a
research prototype, not a safety-certified controller. Keep an
independent emergency-stop method available.

## Repository layout

``` text
scout-hri-robot-system/
├── .gitignore
├── notebook/
│   ├── DTIscount_robot/
│   │   ├── DTIscount_robot/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── scripts/
│   │   ├── package.xml
│   │   └── setup.py
│   └── hri_gui/
│       ├── hri_gui_B0_robust_voice_THAI_ONLY.py
│       ├── hri_gui_B1_R3_THAI_PHONETIC_RECOVERY.py
│       └── research_experiment_logger.py
└── robot/
    └── DTIscount_robot/
        ├── DTIscount_robot/
        ├── config/
        ├── launch/
        ├── package.xml
        └── setup.py
```

-   `notebook/hri_gui/`: operator-side GUI and voice-recognition
    implementations.
-   `hri_gui_B0_robust_voice_THAI_ONLY.py`: baseline voice-recognition
    version.
-   `hri_gui_B1_R3_THAI_PHONETIC_RECOVERY.py`: proposed Thai
    phonetic-recovery version.
-   `research_experiment_logger.py`: experiment logging component.
-   `notebook/DTIscount_robot/`: operator-computer ROS 2 nodes,
    including command relay and human-following code.
-   `robot/DTIscount_robot/`: Raspberry Pi-side ROS 2 package, including
    differential-drive and DDSM motor-driver code.

**Important:** Some same-named files in `notebook/` and `robot/` have
different implementations. Keep them in their respective locations; do
not overwrite one with the other without checking the intended hardware
and communication interface.

## Requirements

The paper reports this experimental setup:

### Operator computer

-   Ubuntu Linux
-   ROS 2 Jazzy
-   Python
-   Faster-Whisper, Whisper Small model, Thai language configuration
-   PyQt-based GUI and microphone/audio input

### Robot computer

-   Raspberry Pi 4
-   ROS 2 Jazzy with Fast DDS
-   Four DDSM drive motors
-   Serial interface `/dev/serial0`, `115200` baud
-   Motor IDs 1--4

The paper does not specify every Python package version, audio-device
index, network setting, or model-cache requirement. Verify these against
the code and your hardware.

## Installation

The commands below assume Ubuntu 24.04 and ROS 2 Jazzy. Run notebook
steps on the operator computer and robot steps on the Raspberry Pi.

### 1. Install ROS 2 Jazzy

If not already installed, follow the official guide:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Source ROS 2 in each terminal:

``` bash
source /opt/ros/jazzy/setup.bash
```

Optional: source it automatically for Bash:

``` bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Clone the repository

``` bash
sudo apt update
sudo apt install -y git
git clone https://github.com/ruttagornP/scout-hri-robot-system.git
cd scout-hri-robot-system
```

If the GitHub owner or repository name changes, use the current
repository URL instead.

### 3. Install build tools

On both computers:

``` bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip python3-venv
```

If rosdep has not been initialized on this computer, run:

``` bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` says it is already initialized, just run
`rosdep update`.

### 4. Build the notebook-side ROS 2 package

On the operator computer:

``` bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/scout_hri_ws/src
ln -sfn ~/scout-hri-robot-system/notebook/DTIscount_robot \
  ~/scout_hri_ws/src/DTIscount_robot

cd ~/scout_hri_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If you cloned the repository somewhere other than your home directory,
adjust the symlink path. If dependencies are missing, inspect
`package.xml` and install the missing dependencies for ROS 2 Jazzy.

### 5. Install voice-GUI dependencies

On the operator computer:

``` bash
sudo apt install -y python3-venv python3-pip portaudio19-dev
python3 -m venv --system-site-packages ~/scout_hri_venv
source ~/scout_hri_venv/bin/activate
python -m pip install --upgrade pip
python -m pip install faster-whisper PyQt5 sounddevice numpy
```

The GUI may use additional modules. If it reports `ModuleNotFoundError`,
install the missing dependency in the active environment after
confirming its package name. Keep ROS/system dependencies in the
appropriate apt/ROS environment rather than indiscriminately installing
everything with pip.

Check audio devices:

``` bash
arecord -l
python -c "import sounddevice as sd; print(sd.query_devices())"
```

Faster-Whisper may download model files the first time it runs. The
Whisper Small model requires additional storage and memory.

### 6. Build the Raspberry Pi-side ROS 2 package

On the Raspberry Pi, install ROS 2 Jazzy and build tools as described
above. Clone the repository on the Pi if it is not already present:

``` bash
git clone https://github.com/ruttagornP/scout-hri-robot-system.git
source /opt/ros/jazzy/setup.bash
mkdir -p ~/scout_robot_ws/src
ln -sfn ~/scout-hri-robot-system/robot/DTIscount_robot \
  ~/scout_robot_ws/src/DTIscount_robot

cd ~/scout_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If the repository is in another directory, update the symlink path.

### 7. Check the serial interface

The paper specifies `/dev/serial0` at 115200 baud for the DDSM motor
driver. On the Raspberry Pi:

``` bash
ls -l /dev/serial0
groups
```

If the serial device is unavailable, check Raspberry Pi serial-port
configuration and permissions. Do not use `sudo` to run the entire ROS
application as a blanket workaround.

## Network and ROS 2 topics

Both computers must be able to communicate over a network that supports
ROS 2 discovery. Use compatible ROS 2 distributions and the same
`ROS_DOMAIN_ID` if one is configured:

``` bash
echo "$ROS_DOMAIN_ID"
```

The paper describes these interfaces:

  Topic/interface         Purpose
  ----------------------- ---------------------------------------
  `/voice_command`        Recognized voice command
  `/cmd_vel`              Linear/angular velocity command
  `/cmd_velocity_left`    Left-side wheel velocity
  `/cmd_velocity_right`   Right-side wheel velocity
  `/dev/serial0`          Serial connection to the motor driver

Check the live ROS 2 graph rather than assuming every topic is
available:

``` bash
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list
ros2 topic info /voice_command
ros2 topic info /cmd_vel
```

ROS 2 discovery may be affected by Wi-Fi client isolation, firewalls,
multicast restrictions, and multiple network interfaces.

## Running the system

### Start robot-side control

On the Raspberry Pi:

``` bash
source /opt/ros/jazzy/setup.bash
source ~/scout_robot_ws/install/setup.bash
ros2 pkg executables DTIscount_robot
```

Inspect the launch-file arguments before running it:

``` bash
ros2 launch DTIscount_robot DTIscount_robot_bringup.launch.py --show-args
```

After checking camera/network settings, serial configuration, and motor
parameters, start the appropriate bringup launch file:

``` bash
ros2 launch DTIscount_robot DTIscount_robot_bringup.launch.py
```

The launch file may start camera streaming and motor-control processes
according to its current contents.

### Start the operator GUI

On the notebook, in a new terminal:

``` bash
source /opt/ros/jazzy/setup.bash
source ~/scout_hri_ws/install/setup.bash
source ~/scout_hri_venv/bin/activate
cd ~/scout-hri-robot-system/notebook/hri_gui
```

Run the proposed Thai phonetic-recovery version:

``` bash
python hri_gui_B1_R3_THAI_PHONETIC_RECOVERY.py
```

Or run the baseline version:

``` bash
python hri_gui_B0_robust_voice_THAI_ONLY.py
```

If the script uses additional local resources or different filenames in
your checkout, follow the configuration and error messages from that
version.

### Verify before moving the robot

First keep the robot immobilized and verify the ROS 2 nodes and topics:

``` bash
ros2 node list
ros2 topic list
ros2 topic info /voice_command
ros2 topic info /cmd_vel
```

Test the voice GUI and inspect topic output as appropriate before
enabling motor actuation. Do not run multiple conflicting motor-control
nodes simultaneously.

## Reproducing the experiment

The paper's evaluation protocol was:

1.  Test FORWARD, BACKWARD, LEFT, RIGHT, and STOP.
2.  Record 20 intentional utterances per command (100 Actual Command
    trials).
3.  Evaluate 100 Non-command/Near-command utterances.
4.  Compare baseline and proposed post-ASR processing with the same ASR
    configuration.
5.  Record recognized commands and processing times.
6.  Report accuracy, FRR, FAR, correct rejection rate (CRR), STOP
    recall, STOP false acceptance, mean latency, median latency, and P90
    latency.

For reproducibility, record the microphone, acoustic environment,
model/language settings, recording procedure, software versions, and any
thresholds or command-rule changes. Respect consent and privacy
requirements for recordings. The repository's `.gitignore` excludes CSV
and audio files by default; publish sanitized sample data only when
intended.

## Safety

-   Test in a controlled area; lift drive wheels for initial motor
    checks.
-   Keep an independent physical emergency stop or motor-power
    disconnect available.
-   Do not rely on voice recognition as the sole emergency-stop
    mechanism.
-   Verify STOP behavior, motor IDs, wheel directions, serial port, and
    command scaling before free-running tests.
-   Keep people clear of the robot's travel path.
-   Never commit tokens, passwords, private recordings, or sensitive
    logs.

The paper reports an increase in FAR from 48% to 52%, and STOP-specific
false acceptance from 7% to 15%. This must be considered before physical
deployment.

## Troubleshooting

### `Package 'DTIscount_robot' not found`

``` bash
source /opt/ros/jazzy/setup.bash
source ~/scout_robot_ws/install/setup.bash
ros2 pkg list | grep DTIscount_robot
```

Use `~/scout_hri_ws/install/setup.bash` on the notebook.

### Python `ModuleNotFoundError`

Activate the GUI environment and inspect the Python interpreter:

``` bash
source ~/scout_hri_venv/bin/activate
which python
python -m pip list
```

Install the missing dependency in that environment after confirming its
package name.

### Microphone unavailable

``` bash
arecord -l
python -c "import sounddevice as sd; print(sd.query_devices())"
```

Check the selected input device and operating-system permissions.

### ROS 2 topics are not visible between computers

Check IP connectivity, ROS 2 distribution compatibility,
`ROS_DOMAIN_ID`, firewall settings, and Wi-Fi multicast/client-isolation
settings.

### Serial device unavailable

``` bash
ls -l /dev/serial0
groups
```

Check Raspberry Pi serial configuration and permissions, and verify the
port and baud rate before starting the motor driver.

## Limitations and future work

The paper identifies future work to reduce false acceptance, improve
latency, and investigate higher-level semantic or intent-based
processing while retaining command constraints and safety checks.

## Citation

If you use this system in research, cite the associated manuscript:

Ruttagorn Prasertlux and Somchat Jiriwibhakorn, "Development of a
Reliable and Responsive Thai Voice Command Recognition System for
Portable Scout Robot Control," *Engineering and Technology Horizons*.

The provided manuscript contains placeholder publication metadata.
Update volume, issue, year, article number, and DOI when the final
bibliographic information is available.

## License and contributions

A license has not yet been specified in this README. Unless a license
file is added, do not assume that the code is licensed for unrestricted
reuse or redistribution.

For bug reports, open a GitHub Issue and include the operating system,
ROS 2 distribution, command run, and relevant error message. Remove
tokens, private IP details, and personal data from logs before posting.
