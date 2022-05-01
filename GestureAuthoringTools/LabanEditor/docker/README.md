# Build docker image
docker build -t labansuite:latest .
# Run docker container
docker run -ti --rm  -e DISPLAY=<hostip:display> labansuite:latest
example of <hostip:display>
192.168.80.1:0.0