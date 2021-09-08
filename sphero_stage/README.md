# sphero_stage

Configuration and launch files for simulating Sphero robots in a simple 2D simulator Stage.

### Requirements

Stage for ROS: `sudo apt install ros-melodic-stage-ros`

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
