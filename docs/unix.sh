#!/bin/bash

# Make sure M.CSS is actually available locally
git submodule update --init --recursive --remote

# Remove old Docker images and output folder
rm -rf ./docs/output
docker image rm m4rqu1705/doxygen_mcss_image:latest

# Use Dockerfile to build image and run container
docker build --tag m4rqu1705/doxygen_mcss_image:latest -f ./docs/Dockerfile .
docker run --name doxygen_mcss_container m4rqu1705/doxygen_mcss_image:latest

# Copy generated website into the AON repository
docker cp doxygen_mcss_container:/root/docs/output/ ./docs/

# Clean up containers after running everything
docker stop doxygen_mcss_container
docker rm doxygen_mcss_container
