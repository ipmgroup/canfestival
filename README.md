This is a fork of the CanFestival-3 project http://dev.automforge.net/CanFestival-3

## This branch only works with Python 2 and is built in Docker.

docker build . -t canfestival

docker-run --name canfestival canfestival

### or

work in devcontainer 

- New example added : examples/thruster_control

### on RPi
./configure --can=socket
