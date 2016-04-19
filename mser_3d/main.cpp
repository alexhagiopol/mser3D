#include "oldTests.h"
#include "InputManager.h"

int main(int argc, char ** argv) {
    syntheticTestOptimization(true,true,true,30);
    /*
    assert(argc == 2 && "Please provide the file path of your settings file e.g. /home/you/mser/Settings.yaml");
    std::string settingsFileLocation = argv[1];
    const InputManager myInput(settingsFileLocation);
    realWorldTestOptimization(myInput, false, 30);
    //testPrintSuperimposedMeasurementImages(myInput);
    */
    return 0;
}
