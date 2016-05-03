#include "oldTests.h"
#include "InputManager.h"
#include "Tracker.h"
#include "visualization.h"

int main(int argc, char ** argv) {
    //Take in input
    assert(argc == 2 && "Please provide the file path of your settings file e.g. /home/you/mser/Settings.yaml");
    std::string settingsFileLocation = argv[1];
    InputManager myInput = InputManager();
    myInput.processSettingsFile(settingsFileLocation);
    
    //Perform synthetic test of optimization math. Performs MSER_3D on synthetic dataset.
    //syntheticTestOptimization(myInput,true,true,true,30);
    //Perform real world test of optimization
    //realWorldTestOptimization(myInput,false,30);

    /*
    std::vector<MserTrack> myTracks;
    myInput.MSERTracks(myTracks);
    Tracker myTracker(myInput);
    myTracker.readImages();
    myTracker.displayTracks(myTracker.images_,myTracks);
     */
    showTextures();
    return 0;
}
