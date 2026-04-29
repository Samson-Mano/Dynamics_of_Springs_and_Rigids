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
;
)";

}


std::string ShaderLibrary::mesh_fragment_shader()
{
    return R"(

#version 330 core
;
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

