//
// Created by alex on 4/22/16.
//

#pragma once
#include "InputManager.h"

class Tracker {
public:
    Tracker(InputManager& input){input_ = input;}
    void observeMSERs();
private:
    InputManager input_;
};

