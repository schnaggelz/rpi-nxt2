#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f "$BASH_SOURCE[0]"))
SOURCE_DIR=$(realpath "${SCRIPT_DIR}/..")

echo "$SOURCE_DIR"

export PYTHONPATH=$SOURCE_DIR/linux/libs/python:$HOME/.local/lib
