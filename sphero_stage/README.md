# sphero_stage

Configuration and launch files for simulating Sphero robots in a simple 2D simulator Stage.

### Requirements

- ROS Noetic & Python 3
- Stage for ROS: `sudo apt install ros-noetic-stage-ros`
- (_optional_) Keyboard teleoperation `sudo apt install ros-noetic-teleop-twist-keyboard`

### Structure
Stage simulator requires three types of files:
- `map.bmp` - Bitmap file describing the map visuals
- `map.yaml`- Configuration file describing the resulution and origin of the map
- `map.world` - File describing the simulation world, including the used map, positions of robots, simulation parameters etc.

Map files are stored in `resources/maps`. Since a new world file must be created every time the number or position of the robots changes, template world files are used to create actual files at runtime. Templates are stored in `resources/world_templates` and created files in `resources/worlds`.

**Adding new maps and world templates:** Create a bitmap representing the desired map and add corresponding .yaml and .world files using the provided ones as reference.

### Usage
1. Specify the map name, number of robots and their initial positions distribution within the map in `launch/launch_params.yaml`.
2. Run the `launch/start.py` script. It will load the launch parameters, create the final world file and launch the simulator, map server, and the node for simulated TF. (If running it directly, remember to start `roscore`. Otherwise, add it to some launch file.)

### Step-by-step example
1. Leave all the values on their default settings.
1. In the 1st terminal: `roscore`
1. In the 2nd terminal: `rosrun sphero_stage start.py` - A window with the simulator should open.
1. In the 3rd terminal: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot_0/cmd_vel` - Now you can control one of the robots by sending keystrokes to the 3rd terminal. Follow onscreen instructions. When done, you can exit with Ctrl-c.
1. In the 3rd terminal (after exiting previous command): `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot_1/cmd_vel` - Now you can control the second robot same as the first one.
1. In the 4th terminal: `rostopic echo /robot_1/odom` - Measured position and velocity of the second robot should be continuously printing out in the terminal.

### Tmuxinator example
The same example as above can be easily done using [Tmuxinator](https://github.com/tmuxinator/tmuxinator). (You must have it installed and also be using custom larics keybindings). By running the command below, multiple terminals will open and run the necessary commands automatically. 
```bash
# Assuming you are in root of the package.
tmuxinator start -p example.yml
```
You can move between terminals by holding the `Ctrl` key and pressing arrow keys. When done, you can press `Ctrl+b` and then `k` to kill all programs and exit.