#include "oldTests.h"
#include "InputManager.h"

int main(int argc, char ** argv) {
    assert(argc == 2);
    std::string settingsFileLocation = argv[1];
    InputManager myInput(settingsFileLocation);
    //testMeasurementFunction();
    syntheticTestOptimization(true,false,20);
    //realWorldTestOptimization(myInput);
    return 0;
}
