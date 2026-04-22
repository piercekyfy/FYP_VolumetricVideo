#version 460 core
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aColor;

out vec3 color;

uniform mat4 mvp;

void main() 
{
	gl_Position =  mvp * vec4(aPosition.x, aPosition.y, aPosition.z, 1.0);
	color = aColor;
}