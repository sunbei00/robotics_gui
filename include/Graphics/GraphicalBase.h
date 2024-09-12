//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_GRAPHICALBASE_H
#define ROBOTICS_GUI_GRAPHICALBASE_H

enum class PRIMITIVE{POINT, LINE, TRIANGLE};

class GraphicalBase{
protected:
    PRIMITIVE primitive;
    unsigned int vao = -1;
    unsigned int vbo = -1;
public:
    GraphicalBase(); // child class, PRIMITIVE, vector(data)
    virtual ~GraphicalBase(); // need to clear vao, vbo
    virtual void draw() = 0;
};

#endif //ROBOTICS_GUI_GRAPHICALBASE_H
