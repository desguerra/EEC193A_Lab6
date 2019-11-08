#/bin/bash
xhost + 127.0.0.1

# Start
docker start chc618_lab6
docker attach chc618_lab6

# Run
# docker run -it --name=chc618_lab6 -v ~/Coding/EEC193A/Lab6:/workspace -e DISPLAY==host.docker.internal:0 eec193a-lab6 /bin/bash
