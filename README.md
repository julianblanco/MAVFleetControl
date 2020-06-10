# MAVFleetControl
This is a wrapper around MAVSDK-Python to simplify control of multiple craft.
It uses threaded asynchronous python to ensure real time tasking of the crafts.

It is intended for use with offboard mode and allows users to create high level interactions and send them to multiple aircraft (UDP or Serial).


## Important Notes

The current version of MAVSDK-Python does not allow for multiple aircraft because a user can not specify the desired port. Please use my fork (https://github.com/julianblanco/MAVSDK-Python) until the PR is hopefully merged.


## Run the examples

Start 2 aircraft in gazebo or your simulator of choice

Start string for gazebo

```
test@gcs-pc:~ cd ~/path/to/px4/firmware/
test@gcs-pc:~ ./gazebo_sitl_multiple_run.sh -m iris -n 2
```

Once the aircraft are spawned, simply run the example 

```
test@gcs-py:~ python3 muliple_aircraft_fly_to_point.py
```
## Architecture

Withing craft.py exists a craft class that allows for crafts to be instantiated.
Each craft is will have the ability to spawn awaitable (asyncio) threads and has locking feautures to allow for diffrent threads and routines to not stomp on eachother. 

Actions are tasked to each aircraft via the add_action method. This adds the action to the aircraft's queue of actions to complete. The key diffrence between this and creating a mission for example is that the "actions" can be user defined and make use of offboard mode. For example the aircraft could be tasked with the following sequence [take off, loiter, go to point, perform loop-de-loop, return home].

Becuase the actions are exposed at the highest level of the code, users can create code to script actions and prevent having to generate lots of manual lines of code and algortimally control the aircraft. (this was all writted so I could avoid using ROS)