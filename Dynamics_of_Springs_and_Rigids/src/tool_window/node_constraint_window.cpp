#include "node_constraint_window.h"

node_constraint_window::node_constraint_window()
{
	// Empty constructor
}

node_constraint_window::~node_constraint_window()
{
	// Empty destructor
}

void node_constraint_window::init()
{
	// Load the texture from a file
	std::string img_path = "./resources/images/frame_supports.png";

	// Bind the constraint image
	LoadTextureFromFile(img_path.c_str(), cnst_image);
}

void node_constraint_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("Nodal Constraints");
	ImGui::Spacing();

	// Options for the Node constraint (Pin Support or Pin Roller Support)
	if (ImGui::RadioButton("Pin Support", selected_constraint_option == 0))
	{
		selected_constraint_option = 0;
	}
	// ImGui::SameLine();
	if (ImGui::RadioButton("Pin Roller Support", selected_constraint_option == 1))
	{
		selected_constraint_option = 1;
	}

	// Display the selected option
	ImGui::Text("Selected Option: %s", (selected_constraint_option == 0) ? "Pin Support" : "Pin Roller Support");

	//_________________________________________________________________________________________
	// Get the constraint angle input
	
	get_constraint_angle_input();

	//_________________________________________________________________________________________

	// Selected Node list
	ImGui::Spacing();

	static char nodeNumbers[1024] = ""; // Increase the buffer size to accommodate more characters

	geom_parameters::copyNodenumberlistToCharArray(selected_nodes, nodeNumbers, 1024);

	ImGui::Text("Selected Nodes: ");
	ImGui::Spacing();

	// Begin a child window with ImGuiWindowFlags_HorizontalScrollbar to enable vertical scrollbar ImGuiWindowFlags_AlwaysVerticalScrollbar
	ImGui::BeginChild("Node Numbers", ImVec2(-1.0f, ImGui::GetTextLineHeight() * 10), true);

	// Assuming 'nodeNumbers' is a char array or a string
	ImGui::TextWrapped("%s", nodeNumbers);

	// End the child window
	ImGui::EndChild();

	//__________________________________________________________________________________________
	// Apply and Delete Button
	// Apply constraint button
	if (ImGui::Button("Apply"))
	{
		apply_nodal_constraint = true; // set the flag to apply to the constraint
	}

	ImGui::SameLine();

	// Delete constraint button
	if (ImGui::Button("Delete"))
	{
		delete_nodal_constraint = true; // set the flag to apply to the constraint
	}

	//__________________________________________________________________________________________
	ImGui::Spacing();

	// Close button
	if (ImGui::Button("Close"))
	{
		// Clear the selected nodes
		this->selected_nodes.clear();
		is_selected_count = false; // Number of selected nodes 0
		is_selection_changed = false; // Set the selection changed

		apply_nodal_constraint = false;
		delete_nodal_constraint = false;
		is_show_window = false; // set the flag to close the window
	}

	//__________________________________________________________________________________________

	// Render reference image
	draw_support();

	ImGui::End();

}

void node_constraint_window::get_constraint_angle_input()
{
	// Input box to give input via text
	static bool input_mode = false;
	static char angle_str[8] = ""; // buffer to store constraint angle string
	static float angle_input = static_cast<float>(constraint_angle); // buffer to store constraint angle value

	// Button to switch to input mode
	if (!input_mode)
	{
		if (ImGui::Button("Constraint Angle"))
		{
			input_mode = true;
			snprintf(angle_str, 8, "%.1f", angle_input); // set the buffer to current angle
		}
	}
	else // input mode
	{
		// Text box to input angle value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##ConstraintAngle", angle_str, IM_ARRAYSIZE(angle_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to float
			angle_input = static_cast<float>(atof(angle_str));
			// limit the value to 0 - 360 range
			angle_input = fmaxf(0.0f, fminf(angle_input, 360.0f));
			// set the angle to input value
			constraint_angle = angle_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			input_mode = false;
		}
	}

	angle_input = static_cast<float>(constraint_angle);
	// Slider for angle
	ImGui::Text("Angle");
	ImGui::SameLine();
	ImGui::SliderFloat("Degrees", &angle_input, 0.0f, 360.0f, "%.1f");

	constraint_angle = angle_input;

	ImGui::Spacing();

}


void node_constraint_window::add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right)
{
	if (is_right == false)
	{
		// Add to the selected node list
		for (int node : selected_nodes)
		{
			// Check whether nodes are already in the list or not
			if (std::find(this->selected_nodes.begin(), this->selected_nodes.end(), node) == this->selected_nodes.end())
			{
				// Add to selected nodes
				this->selected_nodes.push_back(node);

				// Selection changed flag
				this->is_selection_changed = true;
			}
		}
	}
	else
	{
		// Remove from the selected node list
		for (int node : selected_nodes)
		{
			// Erase the node which is found in the list
			this->selected_nodes.erase(std::remove(this->selected_nodes.begin(), this->selected_nodes.end(), node),
				this->selected_nodes.end());

			// Selection changed flag
			this->is_selection_changed = true;
		}

	}

	// Number of selected nodes
	this->is_selected_count = false;
	if (static_cast<int>(this->selected_nodes.size()) > 0)
	{
		this->is_selected_count = true;
	}

}




void node_constraint_window::draw_support()
{
	// Draw the fixed end support image
	if (cnst_image.is_loaded == true)
	{
		ImVec2 window_pos = ImGui::GetCursorScreenPos();
		ImVec2 window_size = ImGui::GetContentRegionAvail();
		ImDrawList* draw_list = ImGui::GetWindowDrawList();


		draw_list->AddRect(window_pos, ImVec2(window_pos.x + window_size.x, window_pos.y + window_size.y), ImColor(255, 255, 255));

		// Initialize zero
		ImVec2 img_pos_top_left(0, 0);
		ImVec2 img_pos_top_right(0, 0);
		ImVec2 img_pos_bot_right(0, 0);
		ImVec2 img_pos_bot_left(0, 0);

		bool draw_img = get_image_min_max_coord(window_pos, window_size, img_pos_top_left, img_pos_top_right,
			img_pos_bot_right, img_pos_bot_left, constraint_angle);

		if (draw_img == true)
		{
			ImVec2 uv_top_left;
			ImVec2 uv_top_right;
			ImVec2 uv_bot_right;
			ImVec2 uv_bot_left;

			if (selected_constraint_option == 0)
			{
				// Draw pin support
				uv_top_left = ImVec2(0.0f, 0.0f);
				uv_top_right = ImVec2(0.5f, 0.0f);
				uv_bot_right = ImVec2(0.5f, 0.5f);
				uv_bot_left = ImVec2(0.0f, 0.5f);
			}
			else if (selected_constraint_option == 1)
			{
				// Draw pin roller support
				uv_top_left = ImVec2(0.5f, 0.0f);
				uv_top_right = ImVec2(1.0f, 0.0f);
				uv_bot_right = ImVec2(1.0f, 0.5f);
				uv_bot_left = ImVec2(0.5f, 0.5f);
			}


			draw_list->AddImageQuad((void*)(intptr_t)cnst_image.image_texture_ID, img_pos_top_left, img_pos_top_right,
				img_pos_bot_right, img_pos_bot_left,
				uv_top_left, uv_top_right, uv_bot_right, uv_bot_left);
		}
	}
}

void node_constraint_window::LoadTextureFromFile(const char* filename, cnst_image_data& cnst_image)
{
	// Private function to Load Texture From File
	// Simple helper function to load an image into a OpenGL texture with common settings

	// Load from file only once
	if (!cnst_image.is_loaded)
	{
		// Load image data from file
		unsigned char* image_data = stbi_load(filename, &cnst_image.image_width, &cnst_image.image_height, NULL, 4);
		if (image_data == NULL)
		{
			cnst_image.is_loaded = false;
			return;
		}

		// Create OpenGL texture
		glGenTextures(1, &cnst_image.image_texture_ID);
		glBindTexture(GL_TEXTURE_2D, cnst_image.image_texture_ID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, cnst_image.image_width, cnst_image.image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
		stbi_image_free(image_data);

		cnst_image.is_loaded = true;
	}
}

bool node_constraint_window::get_image_min_max_coord(ImVec2& window_pos, ImVec2& window_size, 
	ImVec2& img_pos_top_left, ImVec2& img_pos_top_right, 
	ImVec2& img_pos_bot_right, ImVec2& img_pos_bot_left, double& orientation)
{
	// Get the image position based on rotation angle
	if (window_size.x < 50 || window_size.y < 50)
		return false;

	double rect_min_size = std::min(window_size.x, window_size.y);
	const double size_factor = 0.415f;


	// Rectangle origin
	glm::vec2 rect_window_origin = glm::vec2((window_pos.x + window_pos.x + window_size.x) * 0.5f,
		(window_pos.y + window_pos.y + window_size.y) * 0.5f);

	// Corners of the image with origin (0,0)
	glm::vec2 top_left = glm::vec2(-(rect_min_size * size_factor), (rect_min_size * size_factor));
	glm::vec2 top_right = glm::vec2((rect_min_size * size_factor), (rect_min_size * size_factor));
	glm::vec2 bot_right = glm::vec2((rect_min_size * size_factor), -(rect_min_size * size_factor));
	glm::vec2 bot_left = glm::vec2(-(rect_min_size * size_factor), -(rect_min_size * size_factor));

	//___________________________________________________________________________________________________________
	float radians = ((static_cast<float>(orientation) + 90.0f) * 3.14159365f) / 180.0f; // convert degrees to radians
	float cos_theta = cos(radians);
	float sin_theta = sin(radians);

	// Rotated point of the corners
	ImVec2 rotated_pt_top_left = ImVec2((top_left.x * cos_theta) - (top_left.y * sin_theta),
		(top_left.x * sin_theta) + (top_left.y * cos_theta));

	ImVec2 rotated_pt_top_right = ImVec2((top_right.x * cos_theta) - (top_right.y * sin_theta),
		(top_right.x * sin_theta) + (top_right.y * cos_theta));

	ImVec2 rotated_pt_bot_right = ImVec2((bot_right.x * cos_theta) - (bot_right.y * sin_theta),
		(bot_right.x * sin_theta) + (bot_right.y * cos_theta));

	ImVec2 rotated_pt_bot_left = ImVec2((bot_left.x * cos_theta) - (bot_left.y * sin_theta),
		(bot_left.x * sin_theta) + (bot_left.y * cos_theta));

	// Set the corner points after rotating and translating with the origin
	img_pos_top_left = ImVec2(rect_window_origin.x + rotated_pt_top_left.x, rect_window_origin.y + rotated_pt_top_left.y);
	img_pos_top_right = ImVec2(rect_window_origin.x + rotated_pt_top_right.x, rect_window_origin.y + rotated_pt_top_right.y);
	img_pos_bot_right = ImVec2(rect_window_origin.x + rotated_pt_bot_right.x, rect_window_origin.y + rotated_pt_bot_right.y);
	img_pos_bot_left = ImVec2(rect_window_origin.x + rotated_pt_bot_left.x, rect_window_origin.y + rotated_pt_bot_left.y);

	return true;
}
