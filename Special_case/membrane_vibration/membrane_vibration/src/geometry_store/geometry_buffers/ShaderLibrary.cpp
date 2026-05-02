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

