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

	label_shader.createShader(shaderSrc.vertex.c_str(), shaderSrc.fragment.c_str());

	// Set texture uniform variables
	label_shader.Bind();
	label_shader.setUniform("u_Texture", 0);
	label_shader.UnBind();

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
	// Skip if no Character
	if (total_char_count == 0)
	{
		return;
	}

	// Paint all the labels
	label_shader.Bind();
	label_buffers.Bind();
	
	glActiveTexture(GL_TEXTURE0);

	// Bind the texture to the slot
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

	label_shader.Bind();
	label_shader.setUniform("uMVP", mvp, false);
	label_shader.setUniform("uZoomScale", zoomScale);


	glm::vec4 labelColor = glm::vec4(geom_param_ptr->geom_colors.load_color, geom_param_ptr->geom_transparency);
	label_shader.setUniform("uLabelColor", labelColor); // Updating uniforms unBinds shader
	
	label_shader.UnBind();
	
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

	
	// Apply label rotation around the label origin (if any)
	float angleRadians = glm::radians(lb.label_angle);
	glm::mat4 label_rotationMatrix = glm::rotate(glm::mat4(1.0f), angleRadians, glm::vec3(0, 0, 1));

	// Label position
	glm::vec3 labelPos = lb.label_loc;


	float x = labelPos.x - (total_label_width * 0.5f);

	// Whether paint above the location or not
	float y = 0.0f;
	if (lb.label_above_loc == true)
	{
		y = labelPos.y + (total_label_height * 0.5f);
	}
	else
	{
		y = labelPos.y - (total_label_height + (total_label_height * 0.5f));
	}


	for (int i = 0; lb.label[i] != '\0'; ++i)
	{
		// get the atlas information
		char ch = lb.label[i];

		Character ch_data = geom_param_ptr->main_font.ch_atlas[ch];

		float xpos = x + (ch_data.Bearing.x * font_scale);
		float ypos = y - (ch_data.Size.y - ch_data.Bearing.y) * font_scale;

		float w = ch_data.Size.x * font_scale;
		float h = ch_data.Size.y * font_scale;

		float margin = 0.00002f; // This value prevents the minor overlap with the next char when rendering

		// Point 1
		// Vertices [0,0] // 0th point

		vertices[vertex_index + 0] = xpos;
		vertices[vertex_index + 1] = ypos + h;
		vertices[vertex_index + 2] = 0.0;

		// Label origin
		vertices[vertex_index + 3] = labelPos.x;
		vertices[vertex_index + 4] = labelPos.y;
		vertices[vertex_index + 5] = labelPos.z;

		// Texture Glyph coordinate
		vertices[vertex_index + 6] = ch_data.top_left.x + margin;
		vertices[vertex_index + 7] = ch_data.bot_right.y;

		// Iterate
		vertex_index = vertex_index + 8;

		//__________________________________________________________________________________________

		// Point 2
		// Vertices [0,1] // 1th point

		vertices[vertex_index + 0] = xpos;
		vertices[vertex_index + 1] = ypos;
		vertices[vertex_index + 2] = 0.0;

		// Label origin
		vertices[vertex_index + 3] = labelPos.x;
		vertices[vertex_index + 4] = labelPos.y;
		vertices[vertex_index + 5] = labelPos.z;

		// Texture Glyph coordinate
		vertices[vertex_index + 6] = ch_data.top_left.x + margin;
		vertices[vertex_index + 7] = ch_data.top_left.y;

		// Iterate
		vertex_index = vertex_index + 8;

		//__________________________________________________________________________________________

		// Point 3
		// Vertices [1,1] // 2th point

		vertices[vertex_index + 0] = xpos + w;
		vertices[vertex_index + 1] = ypos;
		vertices[vertex_index + 2] = 0.0;

		// Label origin
		vertices[vertex_index + 3] = labelPos.x;
		vertices[vertex_index + 4] = labelPos.y;
		vertices[vertex_index + 5] = labelPos.z;

		// Texture Glyph coordinate
		vertices[vertex_index + 6] = ch_data.bot_right.x - margin;
		vertices[vertex_index + 7] = ch_data.top_left.y;

		// Iterate
		vertex_index = vertex_index + 8;

		//__________________________________________________________________________________________

		// Point 4
		// Vertices [1,0] // 3th point

		vertices[vertex_index + 0] = xpos + w;
		vertices[vertex_index + 1] = ypos + h;
		vertices[vertex_index + 2] = 0.0;

		// Label origin
		vertices[vertex_index + 3] = labelPos.x;
		vertices[vertex_index + 4] = labelPos.y;
		vertices[vertex_index + 5] = labelPos.z;

		// Texture Glyph coordinate
		vertices[vertex_index + 6] = ch_data.bot_right.x - margin;
		vertices[vertex_index + 7] = ch_data.bot_right.y;

		// Iterate
		vertex_index = vertex_index + 8;

		//__________________________________________________________________________________________
		x += (ch_data.Advance >> 6) * font_scale;

		//__________________________________________________________________________________________


		// Fix the index buffers
		// Set the node indices
		unsigned int t_id = ((indices_index / 6) * 4);
		// Triangle 0,1,2
		indices[indices_index + 0] = t_id + 0;
		indices[indices_index + 1] = t_id + 1;
		indices[indices_index + 2] = t_id + 2;

		// Triangle 2,3,0
		indices[indices_index + 3] = t_id + 2;
		indices[indices_index + 4] = t_id + 3;
		indices[indices_index + 5] = t_id + 0;

		// Increment
		indices_index = indices_index + 6;
	}

}



