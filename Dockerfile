# Container image that runs your code
FROM ros:humble

# Copies your code file from your action repository to the filesystem path `/` of the container
COPY entrypoint.sh /entrypoint.sh


RUN apt-get update
RUN apt-get install -q -y libboost-all-dev
# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]