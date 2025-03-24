#!/bin/bash
DIR=$(dirname $(realpath $0))

action=${1:-start}

set -a
source $HOME/.bashrc
source $DIR/config.env
set +a

if [ $action == "start" ]; then

	if [ $NATIVE_MODE == "true" ]; then
		uid=$(id -u) gid=$(id -g) docker-compose -f docker-compose.yml -f docker-compose.native_mode.yml up
	else
		uid=$(id -u) gid=$(id -g) docker-compose up
	fi

elif [ $action == "reset" ]; then
	docker-compose down -v
elif [ $action == "update" ]; then
	docker-compose pull
else
	echo "Usage: run_designer.sh [start|reset|update]"
fi
