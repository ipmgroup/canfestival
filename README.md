This is a fork of the CanFestival-3 project http://dev.automforge.net/CanFestival-3

## This branch only works with Python 2 and is built in Docker.

docker build . -t canfestival

docker-run --name canfestival canfestival

### or

work in devcontainer 

- New example added : [examples/thruster_control](https://github.com/ipmgroup/canfestival/tree/master/examples/thruster_control#readme)

### on RPi
./configure --can=socket

### test branch with python 3
 the project is "build" and the engines are spinning. A full functionality check was not given.
 Use at your own risk. :)
