#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include<GL\glew.h>
#include<GLFW\glfw3.h>


#include "ShaderLibrary.h"	


class Shader
{
private:
    unsigned int m_programID = 0;
    std::unordered_map<std::string, GLint> m_uniformCache;
    std::unordered_map<std::string, GLint> m_attribCache;
    bool m_isBound = false;

    // Debug flag for verbose output
    static bool s_enableDebugOutput;

public:
    Shader();
    ~Shader();

    // Prevent copying
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    // Allow moving
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    // Create shader from source strings
    void createShader(const char* vertexShaderData, const char* fragmentShaderData);
    void createShader(const char* vertexShaderData, const char* fragmentShaderData, const char* geometryShaderData);
    // void createShader(const char* computeShaderData); // Compute shader only

    // Bind/Unbind with state tracking
    void Bind();
    void UnBind();
    bool isBound() const { return m_isBound; }

    // Uniform setters with caching and no unnecessary bind/unbind
    void setUniform(const std::string& name, float value);
    void setUniform(const std::string& name, int value);
    void setUniform(const std::string& name, bool value);
    void setUniform(const std::string& name, const glm::vec2& value);
    void setUniform(const std::string& name, const glm::vec3& value);
    void setUniform(const std::string& name, const glm::vec4& value);
    void setUniform(const std::string& name, const glm::mat3& value, bool transpose = false);
    void setUniform(const std::string& name, const glm::mat4& value, bool transpose = false);

    // Array versions for OpenGL 4.6
    void setUniform(const std::string& name, const std::vector<glm::vec3>& values);
    void setUniform(const std::string& name, const std::vector<glm::mat4>& values);

    // Texture binding (modern approach)
    void setTexture(const std::string& name, GLuint textureID, GLuint unit = 0);
    void setTexture(const std::string& name, GLuint textureID, GLenum target, GLuint unit);

    // Compute shader dispatch
    void dispatchCompute(GLuint numGroupsX, GLuint numGroupsY, GLuint numGroupsZ);
    void dispatchComputeIndirect(GLintptr offset);

    // Utility functions
    GLint getUniformLocation(const std::string& name);
   //  GLint getAttributeLocation(const std::string& name);
    GLuint getProgramID() const { return m_programID; }
    bool isValid() const { return m_programID != 0; }

    // Debug
    void printActiveUniforms() const;
    void printActiveAttributes() const;
    static void setDebugOutput(bool enable) { s_enableDebugOutput = enable; }

private:
    unsigned int compileShader(GLenum type, const char* source, const std::string& shaderName);
    void linkProgram(unsigned int vertexShader, unsigned int fragmentShader, unsigned int geometryShader = 0);
    GLint getCachedUniformLocation(const std::string& name);
};

