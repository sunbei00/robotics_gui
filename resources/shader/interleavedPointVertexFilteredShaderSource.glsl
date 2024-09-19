#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform float zMin;
uniform float zMax;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    if (aPos.z < zMin || aPos.z > zMax) {
        gl_PointSize = 0.0;
    }else{
        gl_PointSize = 1.0;
    }
}