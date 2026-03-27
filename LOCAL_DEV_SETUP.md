# Local Development Setup (PX4 SITL)

This is for local testing with PX4 + Gazebo. The competition will use a different simulator.

## 1. Install Python 3.14.2

```bash
# macOS
brew install pyenv

# Linux
# curl https://pyenv.run | bash

pyenv install 3.14.2
```

## 2. Create virtual environment

```bash
cd Drone_Grand_Prix
~/.pyenv/versions/3.14.2/bin/python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 3. Install PX4 SITL simulator

```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive --force
```

Deactivate conda if you have it, then install build deps in a separate venv:

```bash
conda deactivate
python3 -m venv .venv
source .venv/bin/activate
pip install -r Tools/setup/requirements.txt
```

macOS-specific:

```bash
./Tools/setup/macos.sh --sim-tools
brew install gstreamer qt@5
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:$CMAKE_PREFIX_PATH"
```

## 4. Run the simulator

```bash
cd PX4-Autopilot
source .venv/bin/activate
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:$CMAKE_PREFIX_PATH"
make px4_sitl gz_x500
```

## 5. Test MAVLink connection

In a separate terminal:

```bash
cd Drone_Grand_Prix
source .venv/bin/activate
python connect.py
```

You should see lat/lon/alt telemetry streaming over UDP port 14550.

## Cleanup

After stopping PX4 with Ctrl+C, kill leftover Gazebo processes:

```bash
pkill -f gz; pkill -f ruby
```
