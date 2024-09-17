//
// Created by root on 9/13/24.
//

#ifndef ROBOTICS_GUI_LOADPCD_H
#define ROBOTICS_GUI_LOADPCD_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <cstring>

namespace Utils{
    struct FieldInfo {
        std::string name;
        char type;     // 'F' for float, 'I' for int, 'U' for unsigned int
        int size;      // Size in bytes
        int count;     // Number of elements
    };
    bool loadPCD(const std::string& filename, std::map<std::string, std::vector<float>>& points_data);
}

#endif //ROBOTICS_GUI_LOADPCD_H
