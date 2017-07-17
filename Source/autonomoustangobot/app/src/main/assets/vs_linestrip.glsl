// attributes that are equal for all vertices
uniform mat4 u_MVPMatrix;
uniform vec4 u_LineColor;

// input of vertex shader
attribute vec4 a_Position;

// output of vertex shader
varying vec4 v_PointColor;

// far clipping plane distance
float f = 30.0;
// near clipping plane distance
float n = 0.1;

void main() {
    gl_Position = u_MVPMatrix * a_Position;
    
    // perspective divide
    vec3 ndc = gl_Position.xyz / gl_Position.w ; 
    // to get a linear shading of the rendered lines
    // we transform the ndc depth to a linear one between the near and the far clip plane
    float zDist = 1.0 - ((gl_Position.z - n) / (f - n)); // 1 is near, 0 is far
        
    v_PointColor = u_LineColor * zDist;
}
