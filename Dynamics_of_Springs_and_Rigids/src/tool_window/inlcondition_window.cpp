#include "inlcondition_window.h"

inlcondition_window::inlcondition_window()
{
	// Empty constructor
}

inlcondition_window::~inlcondition_window()
{
	// Empty destructor
}

void inlcondition_window::init()
{
}

void inlcondition_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("Initial Condition");

	ImGui::Text("Initial Displacement");
	// ________________________________________________________________________________________________________________________________
	
	get_idisplx_value_input();
	// ________________________________________________________________________________________________________________________________

	get_idisply_value_input();
	// ________________________________________________________________________________________________________________________________

	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Text("Initial Velocity");

	// ________________________________________________________________________________________________________________________________
	
	get_ivelox_value_input();
	// ________________________________________________________________________________________________________________________________
	
	get_iveloy_value_input();
	// ________________________________________________________________________________________________________________________________
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
		apply_nodal_inlcond = true; // set the flag to apply the initial condition
	}

	ImGui::SameLine();

	// Delete load button
	if (ImGui::Button("Delete"))
	{
		delete_nodal_inlcond = true; // set the flag to delete the initial condition
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

		apply_nodal_inlcond = false;
		delete_nodal_inlcond = false;
		is_show_window = false; // set the flag to close the window
	}

	ImGui::End();
}

void inlcondition_window::add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right)
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

void inlcondition_window::get_idisplx_value_input()
{
	// Input box to give input via text
	static bool displx_input_mode = false;
	static char displx_str[16] = ""; // buffer to store input Displacement X string
	static float displx_input = 0; // buffer to store input Displacement X value

	// Button to switch to input mode
	if (!displx_input_mode)
	{
		if (ImGui::Button("X Displacmenet"))
		{
			displx_input_mode = true;
			snprintf(displx_str, 16, "%.1f", initial_displacement_x); // set the buffer to current Displacement X
		}
	}
	else // input mode
	{
		// Text box to input value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##Displacement X", displx_str, IM_ARRAYSIZE(displx_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			initial_displacement_x = atof(displx_str);
			// set the load value to input value
			// deformation_scale_max = defscale_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			displx_input_mode = false;
		}
	}

	// Text for load value
	ImGui::SameLine();
	ImGui::Text(" %.1f", initial_displacement_x);

}

void inlcondition_window::get_idisply_value_input()
{
	// Input box to give input via text
	static bool disply_input_mode = false;
	static char disply_str[16] = ""; // buffer to store input Displacement Y string
	static float disply_input = 0; // buffer to store input Displacement Y value

	// Button to switch to input mode
	if (!disply_input_mode)
	{
		if (ImGui::Button("Y Displacement"))
		{
			disply_input_mode = true;
			snprintf(disply_str, 16, "%.1f", initial_displacement_y); // set the buffer to current Displacement Y
		}
	}
	else // input mode
	{
		// Text box to input value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##Displacement Y", disply_str, IM_ARRAYSIZE(disply_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			initial_displacement_y = atof(disply_str);
			// set the load value to input value
			// deformation_scale_max = defscale_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			disply_input_mode = false;
		}
	}

	// Text for load value
	ImGui::SameLine();
	ImGui::Text(" %.1f", initial_displacement_y);
}

void inlcondition_window::get_ivelox_value_input()
{
	// Input box to give input via text
	static bool velox_input_mode = false;
	static char velox_str[16] = ""; // buffer to store input Velocity X string
	static float velox_input = 0; // buffer to store input Velocity X value

	// Button to switch to input mode
	if (!velox_input_mode)
	{
		if (ImGui::Button("X Velocity"))
		{
			velox_input_mode = true;
			snprintf(velox_str, 16, "%.1f", initial_velocity_x); // set the buffer to current Velocity X
		}
	}
	else // input mode
	{
		// Text box to input value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##Velocity X", velox_str, IM_ARRAYSIZE(velox_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			initial_velocity_x = atof(velox_str);
			// set the load value to input value
			// deformation_scale_max = defscale_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			velox_input_mode = false;
		}
	}

	// Text for load value
	ImGui::SameLine();
	ImGui::Text(" %.1f", initial_velocity_x);
}

void inlcondition_window::get_iveloy_value_input()
{
	// Input box to give input via text
	static bool veloy_input_mode = false;
	static char veloy_str[16] = ""; // buffer to store input Velocity Y string
	static float veloy_input = 0; // buffer to store input Velocity Y value

	// Button to switch to input mode
	if (!veloy_input_mode)
	{
		if (ImGui::Button("Y Velocity"))
		{
			veloy_input_mode = true;
			snprintf(veloy_str, 16, "%.1f", initial_velocity_y); // set the buffer to current Velocity Y
		}
	}
	else // input mode
	{
		// Text box to input value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##Velocity Y", veloy_str, IM_ARRAYSIZE(veloy_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			initial_velocity_y = atof(veloy_str);
			// set the load value to input value
			// deformation_scale_max = defscale_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			veloy_input_mode = false;
		}
	}

	// Text for load value
	ImGui::SameLine();
	ImGui::Text(" %.1f", initial_velocity_y);
}



