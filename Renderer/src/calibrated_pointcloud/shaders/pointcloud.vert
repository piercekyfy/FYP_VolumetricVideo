#version 460 core
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec2 aTexcoord;

out vec2 texCoord;

uniform mat4 mvp;

void main() 
{
	gl_Position =  mvp * vec4(aPosition.x, aPosition.y, aPosition.z, 1.0);
	texCoord = aTexcoord;
}