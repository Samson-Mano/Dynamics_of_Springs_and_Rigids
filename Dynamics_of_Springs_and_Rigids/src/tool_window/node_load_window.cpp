#include "node_load_window.h"

node_load_window::node_load_window()
{
	// Empty constructor
}

node_load_window::~node_load_window()
{
	// Empty destructor
}

void node_load_window::init()
{
	// Load the texture from a file
	std::string img_path = "./resources/images/load_img.png";

	// Bind the constraint image
	LoadTextureFromFile(img_path.c_str(), load_image);
}

void node_load_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("Loads");

	//_________________________________________________________________________________________
	// Get the load value input

	get_load_value_input();

	//_________________________________________________________________________________________
	// Get the load start time input

	get_load_starttime_input();

	//_________________________________________________________________________________________
	// Get the load end time input

	get_load_endtime_input();

	// Display the Load frequency
	ImGui::Text("Load frequency = %.3f Hz",	1.0f / (load_end_time - load_start_time));

	//_________________________________________________________________________________________
	// Get the load angle input

	get_load_angle_input();

	//_________________________________________________________________________________________
	// Get the load phase angle input

	get_load_phase_angle_input();

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
	// Apply load button
	if (ImGui::Button("Apply"))
	{
		apply_nodal_load = true; // set the flag to apply to the load
	}

	ImGui::SameLine();

	// Delete load button
	if (ImGui::Button("Delete"))
	{
		delete_nodal_load = true; // set the flag to apply to the load
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

		apply_nodal_load = false;
		delete_nodal_load = false;
		is_show_window = false; // set the flag to close the window
	}

	//__________________________________________________________________________________________

	// Render reference image
	draw_load();

	ImGui::End();
}


void node_load_window::get_load_value_input()
{
	// Add Load input controls
	// Input box to give input via text
	static bool loadval_input_mode = false;
	static char load_str[16] = ""; // buffer to store input load string
	static float load_input = static_cast<float>(load_amplitude); // buffer to store input load value

	// Button to switch to input mode
	if (!loadval_input_mode)
	{
		if (ImGui::Button("Input Load"))
		{
			loadval_input_mode = true;
			snprintf(load_str, 16, "%.2f", load_input); // set the buffer to current load value
		}
	}
	else // input mode
	{
		// Text box to input load value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputLoad", load_str, IM_ARRAYSIZE(load_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			load_input = static_cast<float>(atof(load_str));
			// set the load value to input value
			load_amplitude = load_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			loadval_input_mode = false;
		}
	}

	// Text for load value
	ImGui::SameLine();
	ImGui::Text("Load = %.2f", load_input);
}


void node_load_window::get_load_starttime_input()
{
	// Input box to give input via text
	static bool loadstarttime_input_mode = false;
	static char loadstarttime_str[16] = ""; // buffer to store input load string
	static float loadstarttime_input = static_cast<float>(load_start_time); // buffer to store input load start time

	// Button to switch to input mode
	if (!loadstarttime_input_mode)
	{
		if (ImGui::Button("Input Start Time"))
		{
			loadstarttime_input_mode = true;
			snprintf(loadstarttime_str, 16, "%.3f", loadstarttime_input); // set the buffer to current load start time
		}
	}
	else // input mode
	{
		// Text box to input load start time
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputStartTime", loadstarttime_str, IM_ARRAYSIZE(loadstarttime_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			loadstarttime_input = static_cast<float>(atof(loadstarttime_str));
			// set the load start time to input value
			load_start_time = loadstarttime_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			loadstarttime_input_mode = false;
		}
	}

	// Text for load start time
	ImGui::SameLine();
	ImGui::Text("Start Time = %.3f", loadstarttime_input);
}


void node_load_window::get_load_endtime_input()
{
	// Input box to give input via text
	static bool loadendtime_input_mode = false;
	static char loadendtime_str[16] = ""; // buffer to store input load string
	static float loadendtime_input = static_cast<float>(load_end_time); // buffer to store input load End Time

	// Button to switch to input mode
	if (!loadendtime_input_mode)
	{
		if (ImGui::Button("Input End Time"))
		{
			loadendtime_input_mode = true;
			snprintf(loadendtime_str, 16, "%.3f", loadendtime_input); // set the buffer to current load End Time
		}
	}
	else // input mode
	{
		// Text box to input load End Time
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputEndTime", loadendtime_str, IM_ARRAYSIZE(loadendtime_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			loadendtime_input = static_cast<float>(atof(loadendtime_str));
			// set the load End Time to input value
			if (loadendtime_input > load_start_time)
			{
				load_end_time = loadendtime_input;
			}
			else
			{
				loadendtime_input = static_cast<float>(load_end_time);
			}
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			loadendtime_input_mode = false;
		}
	}

	// Text for load End Time
	ImGui::SameLine();
	ImGui::Text("End Time = %.3f", loadendtime_input);
}


void node_load_window::get_load_angle_input()
{
	// Input box to give input via text
	static bool input_mode = false;
	static char angle_str[8] = ""; // buffer to store input angle string
	static float angle_input = static_cast<float>(load_angle); // buffer to store input angle value

	// Button to switch to input mode
	if (!input_mode)
	{
		if (ImGui::Button("Input Angle"))
		{
			input_mode = true;
			snprintf(angle_str, 8, "%.2f", angle_input); // set the buffer to current angle
		}
	}
	else // input mode
	{
		// Text box to input angle value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputAngle", angle_str, IM_ARRAYSIZE(angle_str), ImGuiInputTextFlags_CharsDecimal)) // ImGuiInputTextFlags_CharsDecimal
		{
			// convert the input string to float
			angle_input = static_cast<float>(atof(angle_str));
			// limit the value to 0 - 360 range
			angle_input = fmaxf(0.0f, fminf(angle_input, 360.0f));
			// set the angle to input value
			load_angle = angle_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			input_mode = false;
		}
	}

	// Slider for angle
	angle_input = static_cast<float>(load_angle);

	ImGui::Text("Angle");
	ImGui::SameLine();
	ImGui::SliderFloat("Degrees", &angle_input, 0.0f, 360.0f, "%.2f");

	load_angle = angle_input;
}


void node_load_window::get_load_phase_angle_input()
{
	// Input box to give input via text
	static bool input_mode = false;
	static char phase_angle_str[8] = ""; // buffer to store input phase angle string
	static float phase_angle_input = static_cast<float>(load_phase_angle); // buffer to store input phase angle value

	// Button to switch to input mode
	if (!input_mode)
	{
		if (ImGui::Button("Input Load Phase Angle"))
		{
			input_mode = true;
			snprintf(phase_angle_str, 8, "%.2f", phase_angle_input); // set the buffer to current angle
		}
	}
	else // input mode
	{
		// Text box to input angle value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputPhaseAngle", phase_angle_str, IM_ARRAYSIZE(phase_angle_str), ImGuiInputTextFlags_CharsDecimal)) // ImGuiInputTextFlags_CharsDecimal
		{
			// convert the input string to float
			phase_angle_input = static_cast<float>(atof(phase_angle_str));
			// limit the value to 0 - 360 range
			phase_angle_input = fmaxf(0.0f, fminf(phase_angle_input, 360.0f));
			// set the angle to input value
			load_phase_angle = phase_angle_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			input_mode = false;
		}
	}

	// Slider for angle
	phase_angle_input = static_cast<float>(load_phase_angle);

	ImGui::Text("Angle");
	ImGui::SameLine();
	ImGui::SliderFloat("Phase", &phase_angle_input, 0.0f, 360.0f, "%.2f");

	load_phase_angle = phase_angle_input;

}

void node_load_window::add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right)
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

void node_load_window::draw_load()
{
	// Draw the pin support image
	if (load_image.is_loaded == true)
	{
		ImVec2 window_pos = ImGui::GetCursorScreenPos();
		ImVec2 window_size = ImGui::GetContentRegionAvail();
		ImDrawList* draw_list = ImGui::GetWindowDrawList();


		draw_list->AddRect(window_pos, ImVec2(window_pos.x + window_size.x, window_pos.y + window_size.y),
			ImColor(255, 255, 255));

		ImVec2 img_pos_top_left(0, 0);
		ImVec2 img_pos_top_right(0, 0);
		ImVec2 img_pos_bot_right(0, 0);
		ImVec2 img_pos_bot_left(0, 0);

		double orientation = load_angle;

		if (load_amplitude < 0)
		{
			orientation = orientation + 180.0;
		}

		bool draw_img = get_image_min_max_coord(window_pos, window_size, img_pos_top_left, img_pos_top_right, img_pos_bot_right, img_pos_bot_left, orientation);

		if (draw_img == true)
		{
			draw_list->AddImageQuad((void*)(intptr_t)load_image.image_texture_ID, img_pos_top_left, img_pos_top_right, img_pos_bot_right, img_pos_bot_left);
		}
	}
}

void node_load_window::LoadTextureFromFile(const char* filename, load_image_data& load_image)
{
	// Private function to Load Texture From File
// Simple helper function to load an image into a OpenGL texture with common settings

// Load from file only once
	if (!load_image.is_loaded)
	{
		// Load image data from file
		unsigned char* image_data = stbi_load(filename, &load_image.image_width, &load_image.image_height, NULL, 4);
		if (image_data == NULL)
		{
			load_image.is_loaded = false;
			return;
		}

		// Create OpenGL texture
		glGenTextures(1, &load_image.image_texture_ID);
		glBindTexture(GL_TEXTURE_2D, load_image.image_texture_ID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, load_image.image_width, load_image.image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
		stbi_image_free(image_data);

		load_image.is_loaded = true;
	}
}

bool node_load_window::get_image_min_max_coord(ImVec2& window_pos, ImVec2& window_size, 
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
