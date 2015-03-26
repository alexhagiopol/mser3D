import sys
sys.path.append("/Users/alexhagiopol/Documents/Laboratory/Project_MSER/clearmetrics-master")
sys.path.append("/Users/alexhagiopol/Documents/Laboratory/Project_MSER/PyBioSim-master")
sys.path.append("/Users/alexhagiopol/Documents/Laboratory/Project_MSER/mser_nister")
import clearmetrics
import os
os.system('rm positions_nister.txt')
os.system('rm positions_ground_truth.txt')
import Simulator #Simulator must be imported after removal of file
Simulator.sim.run() #Produce simulator images and txt file with ground truth agent locations.
os.chdir('mser_nister')
os.system('./nister_mser_pipeline.sh')
os.chdir('..')

