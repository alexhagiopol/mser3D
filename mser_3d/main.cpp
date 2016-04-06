#include "oldTests.h"
#include "InputManager.h"

int main(int argc, char ** argv) {
    syntheticTestOptimization(true,true,true,30);
    /*
    assert(argc == 2);
    std::string settingsFileLocation = argv[1];
    InputManager myInput(settingsFileLocation);
    realWorldTestOptimization(myInput, false, 30);
    */
    return 0;
}
