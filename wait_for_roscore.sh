#!/bin/bash

echo "Waiting for roscore to start up..."
until rostopic list &>/dev/null ; do sleep 1; done
echo "Roscore found."
exec "$@"
