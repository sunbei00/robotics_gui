#version 330 core
layout(location = 0) in float x;
layout(location = 1) in float y;
layout(location = 2) in float z;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(x, y, z, 1.0);
    gl_PointSize = 1.0;
}