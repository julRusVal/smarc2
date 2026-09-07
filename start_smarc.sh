#!/bin/bash

# Define the container name
CONTAINER_NAME="smarc2"

# Go to the correct folder
cd ~/smarc2

if [ ! -f "swepos_credentials.env" ]; then
    echo "🚨 ERROR: swepos_credentials.env not found!"
    echo "Please create it with your SWEPOS username and password first."
    exit 1
fi

if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    if [ "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
        echo "Container is already running. Entering..."
        docker exec -it ${CONTAINER_NAME} bash
    else
        echo "Resuming stopped container..."
        docker start -ai ${CONTAINER_NAME}
    fi
else
    echo "Creating a NEW container..."    
    
    docker run -it \
        --name ${CONTAINER_NAME} \
        --network host \
        --env-file swepos_credentials.env \
        --device=/dev/ublox_gps \
	--device=/dev/succorfish \
        -v $(pwd):/home/smarc2user/colcon_ws/src/smarc2 \
        alebax/smarc2:latest
fi
