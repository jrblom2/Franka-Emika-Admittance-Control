#!/bin/bash

DIRECTORY="$1"

rsync -avz --progress "student@station:/home/student/TheWiggler/data/${DIRECTORY}" "./data/"
python3 offline_plots.py ./data/${DIRECTORY}
