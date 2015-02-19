#!/bin/bash
# Nister Shell Script

cd Build
make clean
make nister
./nister ../../images_input/Sequence1/frame01.jpg ../../nister_frame01.jpg