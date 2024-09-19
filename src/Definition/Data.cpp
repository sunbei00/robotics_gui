//
// Created by root on 9/19/24.
//

#include "Definition/Data.h"


namespace DATA {
    Field::Field(unsigned int time, GET_DATA_METHOD method, DATA_TYPE type) : mTime(time), mMethod(method), mType(type){}
}
