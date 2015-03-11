#!/bin/bash
# Nister Shell Script

cd Build
make clean
make nister
./nister ../../PyBioSim-master/images/0.jpg ../../calibration.jpg