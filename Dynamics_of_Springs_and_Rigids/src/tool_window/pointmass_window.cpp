#include "pointmass_window.h"

pointmass_window::pointmass_window()
{
	// Empty constructor
}

pointmass_window::~pointmass_window()
{
	// Empty destructor
}

void pointmass_window::init()
{

}

void pointmass_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("Point Mass");

	// ________________________________________________________________________________________________________________________________
	
	get_massx_value_input();

	// ________________________________________________________________________________________________________________________________
	
	get_massy_value_input();

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
	// Apply Point Mass button
	if (ImGui::Button("Apply"))
	{
		apply_nodal_ptmass = true; // set the flag to apply to the point mass
	}

	ImGui::SameLine();

	// Delete Point Mass button
	if (ImGui::Button("Delete"))
	{
		delete_nodal_ptmass = true; // set the flag to delete to the point mass
	}

	ImGui::Spacing();
	//_________________________________________________________________________________________

	// Close button
	if (ImGui::Button("Close"))
	{
		// Clear the selected nodes
		this->selected_nodes.clear();
		is_selected_count = false; // Number of selected nodes 0
		is_selection_changed = false; // Set the selection changed

		apply_nodal_ptmass = false;
		delete_nodal_ptmass = false;
		is_show_window = false; // set the flag to close the window
	}

	ImGui::End();
}



void pointmass_window::get_massx_value_input()
{
	// Input box to give input via text
	static bool massx_input_mode = false;
	static char massx_str[16] = ""; // buffer to store input Mass X string
	static float massx_input = 0; // buffer to store input Mass X value

	// Button to switch to input mode
	if (!massx_input_mode)
	{
		if (ImGui::Button("Mass X"))
		{
			massx_input_mode = true;
			snprintf(massx_str, 16, "%.1f", mass_x); // set the buffer to current Mass X
		}
	}
	else // input mode
	{
		// Text box to input value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##Mass X", massx_str, IM_ARRAYSIZE(massx_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			mass_x = atof(massx_str);
			// set the load value to input value
			// deformation_scale_max = defscale_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			massx_input_mode = false;
		}
	}

	// Text for load value
	ImGui::SameLine();
	ImGui::Text(" %.1f", mass_x);
}


void pointmass_window::get_massy_value_input()
{
	// Input box to give input via text
	static bool massy_input_mode = false;
	static char massy_str[16] = ""; // buffer to store input Mass Y string
	static float massy_input = 0; // buffer to store input Mass Y value

	// Button to switch to input mode
	if (!massy_input_mode)
	{
		if (ImGui::Button("Mass Y"))
		{
			massy_input_mode = true;
			snprintf(massy_str, 16, "%.1f", mass_y); // set the buffer to current Mass Y
		}
	}
	else // input mode
	{
		// Text box to input value
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##Mass Y", massy_str, IM_ARRAYSIZE(massy_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			mass_y = atof(massy_str);
			// set the load value to input value
			// deformation_scale_max = defscale_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			massy_input_mode = false;
		}
	}

	// Text for load value
	ImGui::SameLine();
	ImGui::Text(" %.1f", mass_y);

}


void pointmass_window::add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right)
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