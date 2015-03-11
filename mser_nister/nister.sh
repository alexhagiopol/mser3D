#!/bin/bash
# Nister Shell Script

cd Build
make clean
make nister
./nister ../../images_input_simulator/0.jpg ../../images_output/simulator_output/100000000.jpg
