//
// Created by root on 9/19/24.
//

#ifndef DATA_H
#define DATA_H

namespace DATA{
    enum class GET_DATA_METHOD{ROS, PCD, OBJ, NONE};
    enum class DATA_TYPE{POINT_CLOUD, MESH, ROBOT, NONE};

    struct Field{
        unsigned int mTime;
        GET_DATA_METHOD mMethod;
        DATA_TYPE mType;

        Field(unsigned int time=0, GET_DATA_METHOD method = GET_DATA_METHOD::NONE, DATA_TYPE type = DATA_TYPE::NONE);
    };
}

#endif //DATA_H
