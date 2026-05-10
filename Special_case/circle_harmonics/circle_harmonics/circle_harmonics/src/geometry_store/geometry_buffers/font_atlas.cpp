#include "font_atlas.h"

font_atlas::font_atlas()
{
}

font_atlas::~font_atlas()
{
}

void font_atlas::create_atlas(std::filesystem::path& resourcePath)
{
    // FreeType initialization
    FT_Library ft;
    if (FT_Init_FreeType(&ft))
    {
        std::cout << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
        return;
    }

    FT_Face face;
    if (FT_New_Face(ft, (resourcePath.string() + "./././resources/fonts/CenturyGothic.ttf").c_str(), 0, &face))
    {
        std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;
        return;
    }

    ch_atlas.clear();
    FT_Set_Pixel_Sizes(face, 0, 128);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // First pass: calculate atlas dimensions
    int atlas_width = 0;
    int atlas_height = 0;

    for (unsigned char c = 0; c < 128; c++)
    {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER))
        {
            std::cout << "ERROR::FREETYPE: Failed to load Glyph" << std::endl;
            continue;
        }
        atlas_width += face->glyph->bitmap.width;
        atlas_height = std::max(atlas_height, static_cast<int>(face->glyph->bitmap.rows));
    }

    // Create a temporary buffer to hold the entire atlas data
    std::vector<unsigned char> atlasData(atlas_width * atlas_height, 0);

    // Generate texture (DON'T upload yet)
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Option 1: Use glTexStorage (preferred for OpenGL 4.2+)
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8, atlas_width, atlas_height);

    // Option 2: Use glTexImage with data (no warning)
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, atlas_width, atlas_height, 0, 
    //              GL_RED, GL_UNSIGNED_BYTE, atlasData.data());

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Fill the atlas data buffer
    int x = 0;
    for (unsigned char c = 0; c < 128; c++)
    {
        if (FT_Load_Char(face, static_cast<char>(c), FT_LOAD_RENDER))
        {
            std::cout << "ERROR::FREETYPE: Failed to load Glyph" << std::endl;
            continue;
        }

        // Copy glyph bitmap to atlas buffer
        int width = face->glyph->bitmap.width;
        int height = face->glyph->bitmap.rows;

        for (int row = 0; row < height; row++)
        {
            int destRow = row;
            int destCol = x;

            // Copy row data
            memcpy(&atlasData[destRow * atlas_width + destCol],
                &face->glyph->bitmap.buffer[row * width],
                width);
        }

        // Store character info
        Character character;
        character.Size = glm::ivec2(width, height);
        character.Bearing = glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top);
        character.Advance = face->glyph->advance.x;

        character.top_left.x = static_cast<float>(x) / static_cast<float>(atlas_width);
        character.top_left.y = 0.0f;
        character.bot_right.x = static_cast<float>(x + width) / static_cast<float>(atlas_width);
        character.bot_right.y = static_cast<float>(height) / static_cast<float>(atlas_height);

        ch_atlas.insert(std::pair<char, Character>(c, character));

        x += width;
    }

    // Upload entire atlas at once (single operation, no warning)
    if (glTexStorage2D) {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, atlas_width, atlas_height,
            GL_RED, GL_UNSIGNED_BYTE, atlasData.data());
    }
    else {
        // For older OpenGL, regenerate the texture with data
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, atlas_width, atlas_height, 0,
            GL_RED, GL_UNSIGNED_BYTE, atlasData.data());
    }

    FT_Done_Face(face);
    FT_Done_FreeType(ft);

}
