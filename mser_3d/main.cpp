#include "tests.h"
#include "InputManager.h"

int main(int argc, char ** argv) {
    assert(argc == 2);
    std::string settingsFileLocation = argv[1];
    InputManager myInput(settingsFileLocation);
    //realWorldTestOptimization(myInput);
    syntheticTestOptimization();
    return 0;
}





