#!/bin/bash

DIRECTORY="$1"

rsync -avz --progress "student@station:/home/student/TheWiggler/${DIRECTORY}" "./"
