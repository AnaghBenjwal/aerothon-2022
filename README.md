# aerothon-2022

## simulator setup
1. Install dronekit, mavproxy, ardupilot, ardupilot-gazebo plugin, and gazebo

2. run the following commands to start the simulation environment
- <code> gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world </code>

- <code> cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console </code>

3. Now run your python script
- <code> python3 [yourpythonscript].py </code>
