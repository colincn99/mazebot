# mazebot

## Demo Video

https://github.com/user-attachments/assets/86c24cf2-9554-47a6-a4d3-22ec3dfbc623

## Description
This is a group Caltech robotics class project.

The project utilizes ROS extensively to perform SLAM in a gazebo environment.
We made heavy use of the gmapping and dwa local planner packages while the global path planner was written by us.



## How to run

Build and run the docker with X11 forwarding. 
If you are on WSL2, you can install VcXSrv and run the following:
```sh
docker build -t mazebot .
docker run -e DISPLAY=$(ip route list default | awk '{print $3}'):0 mazebot
```
