#version 330 core

layout(location = 0) in vec3 vertexPosition_modelspace;

uniform mat4 depthMvp;

void main(){
  gl_Position = depthMvp * vec4(vertexPosition_modelspace, 1);
}
