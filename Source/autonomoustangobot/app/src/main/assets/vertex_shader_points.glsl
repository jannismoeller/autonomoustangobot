// attributes that are equal for all vertices
uniform sampler2D u_ColorTex1D;
uniform mat4 u_MVPMatrix;
uniform float u_OcTreeResolution;

// input of vertex shader
attribute vec4 a_Position;

// output of vertex shader
varying vec4 v_PointColor;

// far clipping plane distance
float f = 30.0;
// near clipping plane distance
float n = 0.1;
// factor with which the resolution of the tree gets multiplied
float pixelPerMeter = 8000.0;

void main(){
    float navigable = a_Position.w;
    
    gl_Position = u_MVPMatrix * vec4(a_Position.xyz, 1); 

    // perspective divide.
    vec3 ndc = gl_Position.xyz / gl_Position.w ;

    float zDist = 1.0 - ndc.z;
    // 1 is at near, 0 at far plane
    gl_PointSize = pixelPerMeter * u_OcTreeResolution * zDist;

    // get depth-corresponding color from look-up-texture
    v_PointColor = texture2D(u_ColorTex1D, vec2(zDist, zDist));
    // for the special cases of visualisation
    if(navigable > 1.0)
    {
        if(navigable == 2.0) {
            // magenat
            v_PointColor = vec4(1, 0, 1, v_PointColor.w);
        }
        else if(navigable == 3.0) {
            // black
            v_PointColor = vec4(0, 0, 0, v_PointColor.w);
        }
    }
    else {
        // node is navigable and might also be a frontiernode
        v_PointColor = vec4(v_PointColor.rg, navigable, v_PointColor.w);
    }
}