#include "label_list_store.h"

label_list_store::label_list_store()
{
	// Empty constructor
}



void label_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameter pointer
	this->geom_param_ptr = geom_param_ptr;

	// Create the label shader
	auto shaderSrc = ShaderLibrary::Get(ShaderLibrary::ShaderType::TextShader);

	label_shader.create_shader_data(shaderSrc.vertex.c_str(), shaderSrc.fragment.c_str());

	// Set texture uniform variables
	label_shader.setUniform("u_Texture", 0);

	// Delete all the labels
	labels.clear();
	total_char_count = 0;
}

void label_list_store::add_text(std::string& label, glm::vec3& label_loc,  double label_angle, bool above_point)
{
	// Create a temporary element
	label_text temp_label;
	temp_label.label = label;
	temp_label.label_loc = label_loc;
	temp_label.label_angle = label_angle;
	temp_label.label_above_loc = above_point;

	// Add to the list
	labels.push_back(temp_label);

	// Add to the char_count
	total_char_count = total_char_count + static_cast<unsigned int>(label.size());
}

void label_list_store::set_buffer()
{
	// Define the label vertices of the model (4 vertex (to form a triangle) 3 position, 3 origin, 2 texture coordinate)
	unsigned int label_vertex_count = 4 * 8 * total_char_count;
	float* label_vertices = new float[label_vertex_count];

	// 6 indices to form a triangle
	unsigned int label_indices_count = 6 * total_char_count;
	unsigned int* label_indices = new unsigned int[label_indices_count];

	unsigned int label_v_index = 0;
	unsigned int label_i_index = 0;

	for (auto& lb : labels)
	{
		// Fill the buffers for every individual character
		get_label_buffer(lb, label_vertices, label_v_index, label_indices, label_i_index);
	}

	// Create a layout
	VertexBufferLayout label_layout;
	label_layout.AddFloat(3);  // World Position
	label_layout.AddFloat(3);  // Label Origin
	label_layout.AddFloat(2); // Texture coordinate

	unsigned int label_vertex_size = label_vertex_count * sizeof(float);

	// Create the buffers
	label_buffers.CreateBuffers(label_vertices, label_vertex_size,
		label_indices, label_indices_count, label_layout);

	// Delete the dynamic array (From heap)
	delete[] label_vertices;
	delete[] label_indices;

}


void label_list_store::paint_text()
{
	// Paint all the labels
	label_shader.Bind();
	label_buffers.Bind();
	
	glActiveTexture(GL_TEXTURE0);
	//// Bind the texture to the slot
	glBindTexture(GL_TEXTURE_2D, geom_param_ptr->main_font.textureID);

	glDrawElements(GL_TRIANGLES, 6 * total_char_count, GL_UNSIGNED_INT, 0);

	glBindTexture(GL_TEXTURE_2D, 0);

	label_buffers.UnBind();
	label_shader.UnBind();
}

void label_list_store::clear_labels()
{
	// Delete all the labels
	labels.clear();
	total_char_count = 0;
}


void label_list_store::update_openGLuniforms()
{


	// Update the shader uniforms for the load shader
	float zoomScale = static_cast<float>(geom_param_ptr->zoom_scale);
	glm::mat4 scalingMatrix = glm::scale(glm::mat4(1.0f),
		glm::vec3(zoomScale, zoomScale, zoomScale));

	glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * scalingMatrix;
	
	//glm::vec3 cameraRight = glm::vec3(viewMatrix[0][0], viewMatrix[1][0], viewMatrix[2][0]);
	//glm::vec3 cameraUp = glm::vec3(viewMatrix[0][1], viewMatrix[1][1], viewMatrix[2][1]);

	// Compute MVP matrix
	glm::mat4 mvp = geom_param_ptr->projectionMatrix *
		viewMatrix *
		geom_param_ptr->rotateTranslation *
		geom_param_ptr->modelMatrix;


	label_shader.setUniform("uMVP", mvp, false);
	label_shader.setUniform("uViewMatrix", viewMatrix, false);


	glm::vec4 labelColor = glm::vec4(geom_param_ptr->geom_colors.load_color, geom_param_ptr->geom_transparency);
	label_shader.setUniform("uLabelColor", labelColor); // Updating uniforms unBinds shader
			
	
}


void label_list_store::get_label_buffer(label_text& lb, float* vertices, unsigned int& vertex_index,
	unsigned int* indices, unsigned int& indices_index)
{
	float font_scale = static_cast<float>(geom_param_ptr->font_size / geom_param_ptr->geom_scale);

	// Find the label total width and total height
	float total_label_width = 0.0f;
	float total_label_height = 0.0f;

	for (int i = 0; lb.label[i] != '\0'; ++i)
	{
		// get the atlas information
		char ch = lb.label[i];
		Character ch_data = geom_param_ptr->main_font.ch_atlas[ch];

		total_label_width += (ch_data.Advance >> 6) * font_scale;
		total_label_height = std::max(total_label_height, ch_data.Size.y * font_scale);
	}

	float zoomScale = static_cast<float>(geom_param_ptr->zoom_scale);
	glm::mat4 scalingMatrix = glm::scale(glm::mat4(1.0f),
		glm::vec3(zoomScale, zoomScale, zoomScale));

	// Note: Matrix4.Transpose in C#
	glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * scalingMatrix;

	// Extract right and up vectors (first two rows)
	glm::vec3 cameraRight = glm::vec3(viewMatrix[0][0], viewMatrix[1][0], viewMatrix[2][0]);
	glm::vec3 cameraUp = glm::vec3(viewMatrix[0][1], viewMatrix[1][1], viewMatrix[2][1]);


	// Apply label rotation around the label origin (if any)
	float angleRadians = glm::radians(lb.label_angle);
	glm::mat4 label_rotationMatrix = glm::rotate(glm::mat4(1.0f), angleRadians, glm::vec3(0, 0, 1));

	// Label position
	glm::vec3 labelPos = lb.label_loc;


	// Calculate Y offset (above or below)
	float yOffset = lb.label_above_loc ? total_label_height * 0.5f : -total_label_height * 1.5f;
	glm::vec3 labelOrigin = labelPos + (cameraUp * yOffset);

	float currentX = -total_label_width * 0.5f;  // Center the text


	for (int i = 0; lb.label[i] != '\0'; ++i)
	{
		// get the atlas information
		char ch = lb.label[i];

		Character ch_data = geom_param_ptr->main_font.ch_atlas[ch];

		float charWidth = (ch_data.Advance >> 6) * font_scale;
		float charHeight = ch_data.Size.y * font_scale;
		float xpos = currentX + (ch_data.Bearing.x * font_scale);
		float ypos = (ch_data.Bearing.y - charHeight) * font_scale;

		float w = charWidth;
		float h = charHeight;
		float margin = 0.00002f;

		// Calculate quad corners in local space (relative to label origin)
		glm::vec3 localCorners[4] = 
		{
			glm::vec3(xpos, ypos + h, 0.0f),      // top-left
			glm::vec3(xpos, ypos, 0.0f),          // bottom-left
			glm::vec3(xpos + w, ypos, 0.0f),      // bottom-right
			glm::vec3(xpos + w, ypos + h, 0.0f)   // top-right
		};

		// Apply local rotation (if any)
		for (int j = 0; j < 4; ++j) 
		{
			localCorners[j] = glm::vec3(label_rotationMatrix * glm::vec4(localCorners[j], 1.0f));
		}

		// Transform to world space using billboarding
		for (int j = 0; j < 4; ++j) 
		{
			glm::vec3 worldPos = labelOrigin +
				cameraRight * localCorners[j].x +
				cameraUp * localCorners[j].y;

			// Store vertex
			vertices[vertex_index + 0] = worldPos.x;
			vertices[vertex_index + 1] = worldPos.y;
			vertices[vertex_index + 2] = worldPos.z;
			vertices[vertex_index + 3] = labelPos.x;  // Origin X
			vertices[vertex_index + 4] = labelPos.y;  // Origin Y
			vertices[vertex_index + 5] = labelPos.z;  // Origin Z

			// UV coordinates
			vertices[vertex_index + 6] = ch_data.top_left.x + margin;

			// Handle Y texture coordinate based on corner
			if (j == 0 || j == 3) 
			{
				vertices[vertex_index + 7] = ch_data.top_left.y;
			}
			else 
			{
				vertices[vertex_index + 7] = ch_data.bot_right.y;
			}

			vertex_index += 8;
		}


		// Indices (two triangles per quad)
		unsigned int baseIdx = (indices_index / 6) * 4;

		indices[indices_index + 0] = baseIdx + 0;
		indices[indices_index + 1] = baseIdx + 1;
		indices[indices_index + 2] = baseIdx + 2;
		indices[indices_index + 3] = baseIdx + 2;
		indices[indices_index + 4] = baseIdx + 3;
		indices[indices_index + 5] = baseIdx + 0;
		indices_index += 6;

		currentX += charWidth;
	}

}



