//
// Created by root on 9/19/24.
//

#ifndef ROBOT_H
#define ROBOT_H

#include "glm.hpp"
#include "detail/type_quat.hpp"


struct Robot{
    glm::vec3 position{0,0,0};
    glm::quat orientation{1,0,0,0};
};




#endif //ROBOT_H
