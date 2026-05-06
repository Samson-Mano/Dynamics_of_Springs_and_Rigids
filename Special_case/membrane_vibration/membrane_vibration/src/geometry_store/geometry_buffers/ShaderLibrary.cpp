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
        ShaderType::ModalRsltShader,
            {
                modalrslt_vertex_shader(),
                modalrslt_fragment_shader()
            }
        },
         {
        ShaderType::PulseRsltShader,
            {
                modalrslt_vertex_shader(),
                modalrslt_fragment_shader()
            }
        },
         {
        ShaderType::LoadViewShader,
            {
                loadview_vertex_shader(),
                loadview_fragment_shader()
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
    uniform vec4 vertexColor;
                    
    layout(location = 0) in vec3 aPosition;
                    

    out vec4 vColor;
                    
    void main()
    {
        gl_Position = uMVP * vec4(aPosition, 1.0);
        vColor = vertexColor;
    }

)";

}


std::string ShaderLibrary::mesh_fragment_shader()
{
    return R"(

    #version 330 core
    
    in vec4 vColor;
    out vec4 fColor;
    
    void main()
    {
        // Simple color output without lighting
        fColor = vColor;
    }
    

)";

}




std::string ShaderLibrary::modalrslt_vertex_shader()
{
    return R"(

    #version 330 core
                    
    // Pre-computed MVP matrix on CPU for better performance
    uniform mat4 uMVP;           // Model-View-Projection matrix
  
    // Deflection parameters
    uniform float uDeflAmplitude = 0.0f; // Sine cycle from animation (-1 to 1) 
    uniform float uDeflScale = 0.0f; // Deflection scale value (controlled by user for visualization)
                
    layout(location = 0) in vec3 aPosition; // UnDeformed Position of Node (Static Position)
    layout(location = 1) in vec3 aDeflValue; // Deflection Value of Node for a particular mode
    layout(location = 2) in float aDeflampl; // Deflection value    
                
    out float vDeflAmplitude;

    void main()
    {
        // Scaled deformed position of the Node
        vec3 deformedPosition = vec3(aPosition.x + (aDeflValue.x * uDeflScale * uDeflAmplitude), 
								    aPosition.y + (aDeflValue.y * uDeflScale * uDeflAmplitude),
								    aPosition.z + (aDeflValue.z * uDeflScale * uDeflAmplitude));

        gl_Position = uMVP * vec4(deformedPosition, 1.0);

        vDeflAmplitude = abs(aDeflampl * uDeflAmplitude);

    }

)";

}


std::string ShaderLibrary::modalrslt_fragment_shader()
{
    return R"(

    #version 330 core
    
    in float vDeflAmplitude;
    out vec4 fColor;
    
    uniform float vTransparency;

    // Jet colormap (blue to red)
    vec3 jetColormap(float value) 
    {
        // Clamp value to [-1, 1] range and remap to [0, 1]
        float t = value; // (clamp(value, -1.0, 1.0) + 1.0) / 2.0;
        
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
        float t = value; // (clamp(value, -1.0, 1.0) + 1.0) / 2.0;
        return vec3(
            sin(t * 3.14159 * 2.0),
            sin((t + 0.33) * 3.14159 * 2.0),
            sin((t + 0.67) * 3.14159 * 2.0)
        ) * 0.5 + 0.5;
    }


    void main()
    {
        // Simple color output without lighting
        vec3 vertexColor = jetColormap(vDeflAmplitude);

        fColor =  vec4(vertexColor, vTransparency); // Set the final color
    }
    

)";

}




std::string ShaderLibrary::pulserslt_vertex_shader()
{
    return R"(

    #version 330 core
                    
    // Pre-computed MVP matrix on CPU for better performance
    uniform mat4 uMVP;           // Model-View-Projection matrix
  
    // Deflection parameters
    uniform float uDeflScale = 0.0f; // Deflection scale value (controlled by user for visualization)
                
    layout(location = 0) in vec3 aPosition; // UnDeformed Position of Node (Static Position)
    layout(location = 1) in vec3 aDeflValue; // Deflection Value of Node for a particular time step
    layout(location = 2) in float aDeflampl; // Deflection value    
                
    out float vDeflAmplitude;

    void main()
    {
        // Scaled deformed position of the Node
        vec3 deformedPosition = vec3(aPosition.x + (aDeflValue.x * uDeflScale), 
								    aPosition.y + (aDeflValue.y * uDeflScale),
								    aPosition.z + (aDeflValue.z * uDeflScale));

        gl_Position = uMVP * vec4(deformedPosition, 1.0);

        vDeflAmplitude = abs(aDeflampl);

    }

)";

}


std::string ShaderLibrary::pulserslt_fragment_shader()
{
    return R"(

    #version 330 core
    
    in float vDeflAmplitude;
    out vec4 fColor;
    
    uniform float vTransparency;

    // Jet colormap (blue to red)
    vec3 jetColormap(float value) 
    {
        // Clamp value to [-1, 1] range and remap to [0, 1]
        float t = value; // (clamp(value, -1.0, 1.0) + 1.0) / 2.0;
        
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
        float t = value; // (clamp(value, -1.0, 1.0) + 1.0) / 2.0;
        return vec3(
            sin(t * 3.14159 * 2.0),
            sin((t + 0.33) * 3.14159 * 2.0),
            sin((t + 0.67) * 3.14159 * 2.0)
        ) * 0.5 + 0.5;
    }


    void main()
    {
        // Simple color output without lighting
        vec3 vertexColor = jetColormap(vDeflAmplitude);

        fColor =  vec4(vertexColor, vTransparency); // Set the final color
    }
    

)";

}


std::string ShaderLibrary::loadview_vertex_shader()
{
    return R"(

     #version 330 core
    
    uniform mat4 uMVP;
    uniform vec4 uVertexColor;
    uniform float uZoomScale = 1.0f;
    
    layout(location = 0) in vec3 aPosition;
    layout(location = 1) in vec3 aOrigin;
    
    out vec4 vColor;
    
    void main()
    {
        // Transform to clip space
        vec4 clipPos = uMVP * vec4(aPosition, 1.0);
        vec4 clipOrigin = uMVP * vec4(aOrigin, 1.0);
        
        // Calculate NDC coordinates
        vec3 ndcPos = clipPos.xyz / clipPos.w;
        vec3 ndcOrigin = clipOrigin.xyz / clipOrigin.w;
        
        // Scale offset in NDC space
        vec2 scaledOffset = (ndcPos.xy - ndcOrigin.xy) / uZoomScale;
        
        // Preserve depth from origin (or position)
        float depth = ndcOrigin.z;
        
        // Final position (back to clip space)
        gl_Position = vec4(ndcOrigin.xy + scaledOffset, depth, 1.0);
        
        vColor = uVertexColor;
    }

)";

}


std::string ShaderLibrary::loadview_fragment_shader()
{
    return R"(


    #version 330 core
    
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

