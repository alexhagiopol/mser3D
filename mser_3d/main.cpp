#include "tests.h"
#include "InputManager.h"

int main(int argc, char ** argv) {
    assert(argc == 2 && "Please provide the file path of your settings file e.g. /home/you/mser/Settings.yaml");
    std::string settingsFileLocation = argv[1];
    InputManager myInput(settingsFileLocation);
    test3DReconstruction(myInput);
    return 0;
}





