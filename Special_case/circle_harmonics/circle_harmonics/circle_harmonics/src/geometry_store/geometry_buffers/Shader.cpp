// Shader.cpp
#include "Shader.h"
#include <iostream>
#include <fstream>
#include <chrono>

bool Shader::s_enableDebugOutput = true;

Shader::Shader() = default;

Shader::~Shader()
{
    if (m_programID != 0) 
    {
        glDeleteProgram(m_programID);
        m_programID = 0;
    }
}


Shader::Shader(Shader&& other) noexcept
    : m_programID(other.m_programID)
    , m_uniformCache(std::move(other.m_uniformCache))
    , m_attribCache(std::move(other.m_attribCache))
    , m_isBound(other.m_isBound)
{
    other.m_programID = 0;
    other.m_isBound = false;
}

Shader& Shader::operator=(Shader&& other) noexcept
{
    if (this != &other) 
    {
        if (m_programID != 0) glDeleteProgram(m_programID);

        m_programID = other.m_programID;
        m_uniformCache = std::move(other.m_uniformCache);
        m_attribCache = std::move(other.m_attribCache);
        m_isBound = other.m_isBound;

        other.m_programID = 0;
        other.m_isBound = false;
    }
    return *this;
}

void Shader::createShader(const char* vertexShaderData, const char* fragmentShaderData)
{
    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderData, "Vertex");
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderData, "Fragment");

    linkProgram(vertexShader, fragmentShader);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    if (s_enableDebugOutput && m_programID != 0) 
    {
        std::cout << "Shader program created successfully. ID: " << m_programID << std::endl;
    }
}

void Shader::createShader(const char* vertexShaderData, const char* fragmentShaderData, const char* geometryShaderData)
{
    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderData, "Vertex");
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderData, "Fragment");
    unsigned int geometryShader = compileShader(GL_GEOMETRY_SHADER, geometryShaderData, "Geometry");

    linkProgram(vertexShader, fragmentShader, geometryShader);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteShader(geometryShader);
}

unsigned int Shader::compileShader(GLenum type, const char* source, const std::string& shaderName)
{
    if (source == nullptr || strlen(source) == 0) 
    {
        std::cerr << "ERROR::SHADER::EMPTY_SHADER_DATA: " << shaderName << std::endl;
        return 0;
    }

    unsigned int shader = glCreateShader(type);
    if (shader == 0) 
    {
        std::cerr << "ERROR::SHADER::COULD_NOT_CREATE_SHADER: " << shaderName << std::endl;
        return 0;
    }

    // Compile shader
    const GLchar* sources[] = { source };
    glShaderSource(shader, 1, sources, nullptr);
    glCompileShader(shader);

    // Check compilation
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) 
    {
        GLchar infoLog[2048];
        glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED: " << shaderName << std::endl;
        std::cerr << infoLog << std::endl;

        // Print shader source for debugging (first 200 chars)
        std::string shaderStr(source);
        std::cerr << "Shader source preview:\n" << shaderStr.substr(0, 200) << "...\n" << std::endl;

        glDeleteShader(shader);
        return 0;
    }

    if (s_enableDebugOutput) 
    {
        std::cout << "Compiled " << shaderName << " shader successfully. ID: " << shader << std::endl;
    }

    return shader;
}

void Shader::linkProgram(unsigned int vertexShader, unsigned int fragmentShader, unsigned int geometryShader)
{
    if (m_programID != 0) 
    {
        glDeleteProgram(m_programID);
        m_uniformCache.clear();
        m_attribCache.clear();
    }

    m_programID = glCreateProgram();

    glAttachShader(m_programID, vertexShader);
    glAttachShader(m_programID, fragmentShader);
    if (geometryShader != 0) 
    {
        glAttachShader(m_programID, geometryShader);
    }

    glLinkProgram(m_programID);

    // Check linking
    GLint success;
    glGetProgramiv(m_programID, GL_LINK_STATUS, &success);
    if (!success) 
    {
        GLchar infoLog[2048];
        glGetProgramInfoLog(m_programID, sizeof(infoLog), nullptr, infoLog);
        std::cerr << "ERROR::SHADER::COULD_NOT_LINK_PROGRAM" << std::endl;
        std::cerr << infoLog << std::endl;
        glDeleteProgram(m_programID);
        m_programID = 0;
        return;
    }

    // Validate program (OpenGL 4.6 feature)
    glValidateProgram(m_programID);
    glGetProgramiv(m_programID, GL_VALIDATE_STATUS, &success);
    if (!success && s_enableDebugOutput) 
    {
        GLchar infoLog[2048];
        glGetProgramInfoLog(m_programID, sizeof(infoLog), nullptr, infoLog);
        std::cout << "Program validation warning: " << infoLog << std::endl;
    }
}

void Shader::Bind()
{
    if (!m_isBound && m_programID != 0) 
    {
        glUseProgram(m_programID);
        m_isBound = true;
    }
}

void Shader::UnBind()
{
    if (m_isBound) 
    {
        glUseProgram(0);
        m_isBound = false;
    }
}

GLint Shader::getCachedUniformLocation(const std::string& name)
{
    auto it = m_uniformCache.find(name);
    if (it != m_uniformCache.end()) 
    {
        return it->second;
    }

    GLint location = glGetUniformLocation(m_programID, name.c_str());
    m_uniformCache[name] = location;

    if (location == -1 && s_enableDebugOutput) 
    {
        std::cout << "Warning: Uniform '" << name << "' not found in shader" << std::endl;
    }

    return location;
}

GLint Shader::getUniformLocation(const std::string& name)
{
    return getCachedUniformLocation(name);
}

void Shader::setUniform(const std::string& name, float value)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) glUniform1f(loc, value);
}

void Shader::setUniform(const std::string& name, int value)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) glUniform1i(loc, value);
}

void Shader::setUniform(const std::string& name, bool value)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) glUniform1i(loc, value ? 1 : 0);
}

void Shader::setUniform(const std::string& name, const glm::vec2& value)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) glUniform2fv(loc, 1, glm::value_ptr(value));
}

void Shader::setUniform(const std::string& name, const glm::vec3& value)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) glUniform3fv(loc, 1, glm::value_ptr(value));
}

void Shader::setUniform(const std::string& name, const glm::vec4& value)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) glUniform4fv(loc, 1, glm::value_ptr(value));
}

void Shader::setUniform(const std::string& name, const glm::mat3& value, bool transpose)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) 
    {
        glUniformMatrix3fv(loc, 1, transpose ? GL_TRUE : GL_FALSE, glm::value_ptr(value));
    }
}

void Shader::setUniform(const std::string& name, const glm::mat4& value, bool transpose)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) 
    {
        glUniformMatrix4fv(loc, 1, transpose ? GL_TRUE : GL_FALSE, glm::value_ptr(value));
    }
}

void Shader::setUniform(const std::string& name, const std::vector<glm::vec3>& values)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1 && !values.empty()) 
    {
        glUniform3fv(loc, static_cast<GLsizei>(values.size()), glm::value_ptr(values[0]));
    }
}

void Shader::setUniform(const std::string& name, const std::vector<glm::mat4>& values)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1 && !values.empty()) 
    {
        glUniformMatrix4fv(loc, static_cast<GLsizei>(values.size()), GL_FALSE, glm::value_ptr(values[0]));
    }
}

void Shader::setTexture(const std::string& name, GLuint textureID, GLuint unit)
{
    setTexture(name, textureID, GL_TEXTURE_2D, unit);
}

void Shader::setTexture(const std::string& name, GLuint textureID, GLenum target, GLuint unit)
{
    GLint loc = getCachedUniformLocation(name);
    if (loc != -1) 
    {
        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(target, textureID);
        glUniform1i(loc, unit);
    }
}

void Shader::dispatchCompute(GLuint numGroupsX, GLuint numGroupsY, GLuint numGroupsZ)
{
    if (m_programID != 0) 
    {
        Bind();
        glDispatchCompute(numGroupsX, numGroupsY, numGroupsZ);
        glMemoryBarrier(GL_ALL_BARRIER_BITS);
    }
}

void Shader::dispatchComputeIndirect(GLintptr offset)
{
    if (m_programID != 0) 
    {
        Bind();
        glDispatchComputeIndirect(offset);
        glMemoryBarrier(GL_ALL_BARRIER_BITS);
    }
}

void Shader::printActiveUniforms() const
{
    if (m_programID == 0) return;

    GLint numUniforms = 0;
    glGetProgramiv(m_programID, GL_ACTIVE_UNIFORMS, &numUniforms);

    std::cout << "Active uniforms (" << numUniforms << "):" << std::endl;

    for (GLint i = 0; i < numUniforms; ++i) 
    {
        GLchar name[256];
        GLsizei length;
        GLint size;
        GLenum type;

        glGetActiveUniform(m_programID, i, sizeof(name), &length, &size, &type, name);
        GLint location = glGetUniformLocation(m_programID, name);

        std::cout << "  " << i << ": " << name << " (loc=" << location
            << ", size=" << size << ", type=" << type << ")" << std::endl;
    }
}

void Shader::printActiveAttributes() const
{
    if (m_programID == 0) return;

    GLint numAttributes = 0;
    glGetProgramiv(m_programID, GL_ACTIVE_ATTRIBUTES, &numAttributes);

    std::cout << "Active attributes (" << numAttributes << "):" << std::endl;

    for (GLint i = 0; i < numAttributes; ++i) 
    {
        GLchar name[256];
        GLsizei length;
        GLint size;
        GLenum type;

        glGetActiveAttrib(m_programID, i, sizeof(name), &length, &size, &type, name);
        GLint location = glGetAttribLocation(m_programID, name);

        std::cout << "  " << i << ": " << name << " (loc=" << location
            << ", size=" << size << ", type=" << type << ")" << std::endl;
    }
}



