# MAVFleetControl
This is a wrapper around MAVSDK-Python to simplify control of multiple craft
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
