#!/bin/bash

# Get working directory
full_path=$(realpath $0)
dir_path=$(dirname $full_path)

# Generate personal ini
echo -e "[Plugins]
Plugin\\DockArea=1
Plugin\\Filename=libPlugin
Plugin\\Path=${dir_path}/../libs
Plugin\\Visible=true" > RobWorkStudio.ini
