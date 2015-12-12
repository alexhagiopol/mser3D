MSER
====

This repository stores the Borg Lab's work on Maximally Stable Extremal Regions extraction, association, and 3D projection. It is divided into two main folders.

mser_2d 
-------

This directory contains Matlab code that processes input videos, performs MSER region detection, and associates detected regions according to a nearest neighbor algorithm.

mser_3d
-------

This directory contains C++ code that takes as input the associated regions from mser_2d and uses these associations to project 3D objects into a virtual scene.

Prerequisites
-------------

 1. Matlab
 2. VL Feat: Download and copy folder into the top level directory of this project.
 3. GTSAM: Download and install according to its instructions.
 4. Extra video drivers/software for the Linux version of Matlab:
  a. sudo add-apt-repository ppa:mc3man/trusty-media 
  b. sudo apt-get install ffmpeg gstreamer0.10-ffmpeg
  c. sudo apt-get install libgstreamer0.10-dev gstreamer-tools gstreamer0.10-tools gstreamer0.10-doc
  d. sudo apt-getinstall gstreamer0.10-plugins-base gstreamer0.10-plugins-good gstreamer0.10-plugins-ugly gstreamer0.10-plugins-bad gstreamer0.10-plugins-bad-multiverse