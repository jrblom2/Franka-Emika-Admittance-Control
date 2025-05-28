#!/bin/bash
rsync -a --no-owner --no-group --progress \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  /home/joe/final/TheWiggler/ \
  student@station:/home/student/TheWiggler