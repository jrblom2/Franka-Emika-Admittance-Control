#!/bin/bash

DIRECTORY="$1"

if [ "$DIRECTORY" == "latest" ]; then
    # Get the latest directory matching the pattern data_YYYY-MM-DD_HH-MM-SS
    DIRECTORY=$(ssh student@station 'ls -1 /home/student/TheWiggler/data/ | grep -E "^data_[0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2}$" | sort | tail -n 1')

    if [ -z "$DIRECTORY" ]; then
        echo "No timestamped directories found on remote."
        exit 1
    fi

    echo "Latest directory determined to be: $DIRECTORY"
fi

# Fetch the directory using rsync
rsync -avz --progress "student@station:/home/student/TheWiggler/data/${DIRECTORY}" "./data/"

# Run the Python script
python3 offline_plots.py "./data/${DIRECTORY}"
