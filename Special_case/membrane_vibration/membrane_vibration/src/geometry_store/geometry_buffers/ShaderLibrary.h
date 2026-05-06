#pragma once
#include <string>
#include <unordered_map>



class ShaderLibrary
{
public:
    enum class ShaderType
    {
        MeshShader,
        ModalRsltShader,
        PulseRsltShader,
        LoadViewShader,
        TextShader
    };

    struct ShaderSource
    {
        std::string vertex;
        std::string fragment;
    };


    ShaderLibrary();
    ~ShaderLibrary() = default;

    static const ShaderSource& Get(ShaderType type);


private:
    static std::unordered_map<ShaderType, ShaderSource>& GetLibrary();

    // Mesh shader
    static std::string mesh_vertex_shader();
    static std::string mesh_fragment_shader();

    // Modal Rslt shader
    static std::string modalrslt_vertex_shader();
    static std::string modalrslt_fragment_shader();

    // Pulse Rslt shader
    static std::string pulserslt_vertex_shader();
    static std::string pulserslt_fragment_shader();

    // Load View Shader
    static std::string loadview_vertex_shader();
    static std::string loadview_fragment_shader();

    // Text shader
    static std::string text_vertex_shader();
    static std::string text_fragment_shader();


};



