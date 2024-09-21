#version 330 core
layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform float lineWidth;
uniform mat4 view;
uniform mat4 projection;

void main() {
    vec4 p0 = gl_in[0].gl_Position;
    vec4 p1 = gl_in[1].gl_Position;

    vec2 lineDir = normalize((p1.xy / p1.w) - (p0.xy / p0.w));
    vec2 offsetDir = vec2(-lineDir.y, lineDir.x);
    vec2 offset = offsetDir * lineWidth / 2.0;

    gl_Position = p0 + projection * vec4(offset, 0.0, 0.0);
    EmitVertex();
    gl_Position = p0 - projection * vec4(offset, 0.0, 0.0);
    EmitVertex();

    gl_Position = p1 + projection * vec4(offset, 0.0, 0.0);
    EmitVertex();
    gl_Position = p1 - projection * vec4(offset, 0.0, 0.0);
    EmitVertex();

    EndPrimitive();
}
