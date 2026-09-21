#!/bin/sh
# Exercises the RotorHazard node protocol and lap detection on the host, with
# the ESP-IDF headers rhnode.c needs stubbed out under stub/.
set -e
cd "$(dirname "$0")"
cc -O1 -Wall -Werror -I stub -I ../main -o /tmp/rhnode_test rhnode_test.c ../main/rhnode.c
exec /tmp/rhnode_test
