#version 330 core
out vec4 FragColor;
in vec3 ourColor;
in float alphaOut;

void main()
{
    FragColor = vec4(ourColor, alphaOut);
}