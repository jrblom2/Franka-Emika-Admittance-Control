#!/bin/bash
rsync -a --no-owner --no-group --progress --exclude 'build/' /home/joe/final/TheWiggler/ student@station:/home/student/TheWiggler
