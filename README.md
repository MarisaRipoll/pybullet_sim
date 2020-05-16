# Pybullet_sim

## Running the scripts:

cd into the directory with the python files and source your terminal.

```bash
cd pybullet_sim/src
source /path/to/ros/eloquent/local_setup.bash
```

## Human figure:

- For URDF version:

```bash
python3 human_moveit.py
```

- For MJCF version:
```bash
python3 human_mjcf.py
```

## Human with string attached to hand:

- For URDF version:

```bash
python3 human_string_moveit.py
```

- For MJCF version:
```bash
python3 human_string_mjcf.py
```

To see the implementation error of the setJointMotorControlArray uncomment the line before the last loop in human_string_moveit.py


## Trying to pull the string:

- For URDF version:
```bash
python3 human_pull_string_moveit.py
```

- For MJCF version:
```bash
python3 human_pull_string_mjcf.py
```
