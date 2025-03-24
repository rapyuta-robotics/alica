#!/bin/bash
DIR=$(dirname $(realpath $0))

action=${1:-start}

export SOCIAL_APP_CLIENT_ID=Ov23li6CwLED7raY6S4M
export SOCIAL_APP_SECRET=f8775eb0f4cf412bdabccbe9a5846a6b4633651a

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
