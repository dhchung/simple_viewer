#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 ourColor;
out float alphaOut;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 input_color;

uniform int color_bool;

uniform float color_alpha;

void main(){
    gl_Position = projection * view * model * vec4(aPos, 1.0f);
    if(color_bool==0) {
        ourColor = aColor;
    }else{
        ourColor = input_color;
    }
    alphaOut = color_alpha;
}