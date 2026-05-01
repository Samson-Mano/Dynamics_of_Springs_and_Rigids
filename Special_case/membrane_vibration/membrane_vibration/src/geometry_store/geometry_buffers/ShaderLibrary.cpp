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

    uniform mat4 modelMatrix;
    uniform mat4 viewMatrix;
    uniform mat4 projectionMatrix;
    uniform vec3 vertexColor;

    uniform mat4 panTranslation;
    uniform mat4 rotateTranslation;
    uniform float zoomscale = 1.0f;

    uniform float transparency = 1.0f;
    uniform float geom_scale;

    layout(location = 0) in vec3 node_position;
    layout(location = 1) in vec3 vertexNormal;

    out vec4 v_Color;

    void main()
    {
        // apply zoom scaling and Rotation to model matrix
        mat4 scalingMatrix = mat4(1.0) * zoomscale;
        scalingMatrix[3][3] = 1.0f;
        mat4 scaledModelMatrix = rotateTranslation * scalingMatrix * modelMatrix;

        // apply Translation to the final position 
        vec4 finalPosition = scaledModelMatrix * vec4(node_position, 1.0f) * panTranslation;

        // Final position passed to fragment shader
        gl_Position = finalPosition;

        // v_Color = vec4(vertexColor, transparency);
        v_Color = vec4(0.0, 0.0, 1.0, 1.0);

    }

)";

}


std::string ShaderLibrary::mesh_fragment_shader()
{
    return R"(

#version 330 core

in vec4 v_Color;

out vec4 f_Color; // fragment's final color (out to the fragment shader)

void main()
{
	f_Color = v_Color;
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

