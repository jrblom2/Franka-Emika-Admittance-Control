#!/bin/bash
rsync -a --no-owner --no-group --progress \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude 'data/' \
  --exclude 'src/data_analysis/' \
  --exclude 'src/ergodic/' \
  /home/joe/final/TheWiggler/ \
  student@station:/home/student/TheWiggler