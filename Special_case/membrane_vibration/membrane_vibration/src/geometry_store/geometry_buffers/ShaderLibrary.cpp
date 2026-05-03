#include "ShaderLibrary.h"

ShaderLibrary::ShaderLibrary()
{
    // Empty constructor
}

const ShaderLibrary::ShaderSource& ShaderLibrary::Get(ShaderType type)
{
    return GetLibrary().at(type);
}

std::unordered_map<ShaderLibrary::ShaderType, ShaderLibrary::ShaderSource>& ShaderLibrary::GetLibrary()
{
    static std::unordered_map<ShaderType, ShaderSource> library =
    {
        {
            ShaderType::MeshShader,
            {
                mesh_vertex_shader(),
                mesh_fragment_shader()
            }
        },
                {
        ShaderType::MeshRsltShader,
            {
                meshrslt_vertex_shader(),
                meshrslt_fragment_shader()
            }
        },
        {
            ShaderType::TextShader,
            {
                text_vertex_shader(),
                text_fragment_shader()
            }
        }
    };

    return library;
}



std::string ShaderLibrary::mesh_vertex_shader()
{
    return R"(

    #version 330 core
                    
    // Pre-computed MVP matrix on CPU for better performance
    uniform mat4 uMVP;           // Model-View-Projection matrix
    uniform mat4 uNormalMatrix;  // For normal transformation
    uniform vec4 vertexColor;
                    
    layout(location = 0) in vec3 aPosition;
    layout(location = 1) in vec3 aNormal;
                    
    out vec3 vNormal;
    out vec4 vColor;
                    
    void main()
    {
        gl_Position = uMVP * vec4(aPosition, 1.0);
        vNormal = normalize(mat3(uNormalMatrix) * aNormal);
        vColor = vertexColor;
    }

)";

}


std::string ShaderLibrary::mesh_fragment_shader()
{
    return R"(

  #version 330 core
    
    in vec3 vNormal;
    in vec4 vColor;
    out vec4 fColor;
    
    void main()
    {
        // Simple color output without lighting
        fColor = vColor;
    }
    

)";

}




std::string ShaderLibrary::meshrslt_vertex_shader()
{
    return R"(

    #version 330 core
                    
    // Pre-computed MVP matrix on CPU for better performance
    uniform mat4 uMVP;           // Model-View-Projection matrix
    uniform mat4 uNormalMatrix;  // For normal transformation
    uniform mat4 uModelMatrix;   // For deformation in local space

    // Deformation parameters
    uniform float uDeflScale = 0.0f; 
                
    layout(location = 0) in vec3 aPosition; // UnDeformed Position of Node (Static Position)
    layout(location = 1) in vec3 aDeflValue; // Deformation Value of Node at time t
    layout(location = 2) in vec3 aNormal;
    layout(location = 3) in float aDeflAmplitude; // Normalized deflection value (0.0 to 1.0)
                    
    out vec3 vNormal;
    out float vDeflAmplitude;
    out vec3 vWorldPosition;

    void main()
    {
        // Scaled deformed position of the Node
        vec3 deformedPosition = vec3(aPosition.x + (aDeflValue.x * uDeflScale), 
								    aPosition.y + (aDeflValue.y * uDeflScale),
								    aPosition.z + (aDeflValue.z * uDeflScale));

        gl_Position = uMVP * vec4(deformedPosition, 1.0);
        vNormal = normalize(mat3(uNormalMatrix) * aNormal);
        vDeflAmplitude = aDeflAmplitude;

        vec4 worldPos = uModelMatrix * vec4(deformedPosition, 1.0);
        vWorldPosition = worldPos.xyz;
    }

)";

}


std::string ShaderLibrary::meshrslt_fragment_shader()
{
    return R"(

  #version 330 core
    
    in vec3 vNormal;
    in float vDeflscale;
    out vec4 fColor;
    
    uniform float vTransparency;

    // Jet colormap (blue to red)
    vec3 jetColormap(float value) 
    {
        // Clamp value to [-1, 1] range and remap to [0, 1]
        float t = (clamp(value, -1.0, 1.0) + 1.0) / 2.0;
        
        // Jet colormap algorithm
        vec3 color;
        color.r = clamp(1.5 - abs(4.0 * t - 3.0), 0.0, 1.0);
        color.g = clamp(1.5 - abs(4.0 * t - 2.0), 0.0, 1.0);
        color.b = clamp(1.5 - abs(4.0 * t - 1.0), 0.0, 1.0);
        
        return color;
    }
    
    // Rainbow colormap
    vec3 rainbowColormap(float value)
    {
        float t = (clamp(value, -1.0, 1.0) + 1.0) / 2.0;
        return vec3(
            sin(t * 3.14159 * 2.0),
            sin((t + 0.33) * 3.14159 * 2.0),
            sin((t + 0.67) * 3.14159 * 2.0)
        ) * 0.5 + 0.5;
    }


    void main()
    {
        // Simple color output without lighting
        vec3 vertexColor = jetColormap(interpolated_defl);

        fColor =  vec4(vertexColor, vTransparency); // Set the final color
    }
    

)";

}




std::string ShaderLibrary::text_vertex_shader()
{
    return R"(

#version 330 core
)";

}


std::string ShaderLibrary::text_fragment_shader()
{
    return R"(

#version 330 core

)";

}

