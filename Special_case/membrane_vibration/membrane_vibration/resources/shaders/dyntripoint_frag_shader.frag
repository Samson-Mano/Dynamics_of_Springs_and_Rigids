#version 330 core

in vec4 v_Color;
in vec4 e025_Color;
in vec4 e050_Color;
in vec4 e075_Color;

out vec4 f_Color; // fragment's final color (out to the fragment shader)

void main() {
    // Barycentric coordinates
    vec3 barycentric = vec3(1.0 - gl_FragCoord.y / gl_FragCoord.w - gl_FragCoord.x / gl_FragCoord.w,
                             gl_FragCoord.y / gl_FragCoord.w,
                             gl_FragCoord.x / gl_FragCoord.w);
    
    // Interpolate the fragment color using barycentric coordinates
    // vec4 interpolatedColor = vec4(v_Color.rgb * barycentric.x, v_Color.a) +
       //                      vec4(e025_Color.rgb * barycentric.y, e025_Color.a) +
       //                      vec4(e050_Color.rgb * barycentric.z, e050_Color.a) +
       //                      vec4(e075_Color.rgb * (1.0 - barycentric.y - barycentric.z), e075_Color.a);


     vec4 interpolatedColor = vec4(v_Color.rgb * barycentric.x, v_Color.a) +
                             vec4(e025_Color.rgb * barycentric.y, e025_Color.a) +
                             vec4(e050_Color.rgb * barycentric.z, e050_Color.a) +
                             vec4(e075_Color.rgb * (1.0 - barycentric.y - barycentric.z), e075_Color.a);

    // https://stackoverflow.com/questions/18035719/drawing-a-border-on-a-2d-polygon-with-a-fragment-shader
    
    f_Color = interpolatedColor; // Set the final color
}