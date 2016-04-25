#include "oldTests.h"
#include "InputManager.h"

int main(int argc, char ** argv) {
    //Take in input
    assert(argc == 2 && "Please provide the file path of your settings file e.g. /home/you/mser/Settings.yaml");
    std::string settingsFileLocation = argv[1];
    InputManager myInput = InputManager();
    myInput.processSettingsFile(settingsFileLocation);

    //Perform synthetic test of optimization math. Performs MSER_3D on synthetic dataset.
    syntheticTestOptimization(myInput,true,true,true,30);

    //Perform MSER_3D on real-world data.
    return 0;
}
