#include "oldTests.h"
#include "InputManager.h"

int main(int argc, char ** argv) {
    assert(argc == 2);
    std::string settingsFileLocation = argv[1];
    InputManager myInput(settingsFileLocation);
    syntheticTestOptimization(true,true,false,30);
    //realWorldTestOptimization(myInput, false, 30);
    return 0;
}
