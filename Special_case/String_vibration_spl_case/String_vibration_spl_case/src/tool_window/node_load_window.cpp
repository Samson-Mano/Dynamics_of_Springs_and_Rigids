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
	is_show_window = false;
	execute_apply_load = false;
	execute_remove_load = false;
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

	ImGui::Spacing();

	//_____________________________________________________________________________________________________________________________________________________________________
	// Create four radio buttons for interpolation options

	ImGui::RadioButton("Linear Load Interpolation", &node_load_type, 0);
	ImGui::RadioButton("Cubic Bezier Load Interpolation", &node_load_type, 1);
	ImGui::RadioButton("Sine Load Interpolation", &node_load_type, 2);
	ImGui::RadioButton("Rectangular Load Interpolation", &node_load_type, 3);
	ImGui::RadioButton("Single Node Load", &node_load_type, 4);

	ImGui::Spacing();


	//_________________________________________________________________________________________
	// Get the load start node

	get_load_Start_Node();

	//_________________________________________________________________________________________
	// Get the load end node

	if (node_load_type != 4)
	{
		get_load_End_Node();
	}

	//__________________________________________________________________________________________
	// Apply and Delete Button
	// Apply load button
	if (ImGui::Button("Apply"))
	{
		execute_apply_load = true; // set the flag to apply to the load
	}

	ImGui::SameLine();

	// Delete load button
	if (ImGui::Button("Delete"))
	{
		execute_remove_load = true; // set the flag to apply to the load
	}

	//__________________________________________________________________________________________
	ImGui::Spacing();

	// Close button
	if (ImGui::Button("Close"))
	{
		is_show_window = false;
		execute_apply_load = false;
		execute_remove_load = false;
	}

	//__________________________________________________________________________________________

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



void node_load_window::get_load_Start_Node()
{
	// Text for Load At Start Node
		//_________________________________________________________________________________________
		// Input box to give input via text
	static bool nodeLoadStartnode_input_mode = false;
	static char nodeLoadStartnode_str[16] = ""; // buffer to store input Load At Node string
	static int nodeLoadStartnode_input = static_cast<int>(node_load_start); // buffer to store input Load At Node

	// Button to switch to input mode
	if (!nodeLoadStartnode_input_mode)
	{
		if (ImGui::Button("Load Start Node"))
		{
			nodeLoadStartnode_input_mode = true;
			snprintf(nodeLoadStartnode_str, 16, "%i", nodeLoadStartnode_input); // set the buffer to current Load At Node
		}
	}
	else // input mode
	{
		// Text box to input Load At Node
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputLoadStartNode", nodeLoadStartnode_str, IM_ARRAYSIZE(nodeLoadStartnode_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			nodeLoadStartnode_input = static_cast<int>(atoi(nodeLoadStartnode_str));
			// set the Load At Node to input value
			node_load_start = nodeLoadStartnode_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			nodeLoadStartnode_input_mode = false;
		}
	}

	if (node_load_start >= node_load_end)
	{
		node_load_end = node_load_start + 1;
	}

	// Text for Load At Node
	ImGui::SameLine();
	ImGui::Text("Nodal Load Start Node = %i", node_load_start);

}


void node_load_window::get_load_End_Node()
{
	// Text for Load At End Node
	//_________________________________________________________________________________________
	// Input box to give input via text
	static bool nodeLoadEndnode_input_mode = false;
	static char nodeLoadEndnode_str[16] = ""; // buffer to store input Load End Node string
	static int nodeLoadEndnode_input = static_cast<int>(node_load_end); // buffer to store input Load End Node

	// Button to switch to input mode
	if (!nodeLoadEndnode_input_mode)
	{
		if (ImGui::Button("Load End Node"))
		{
			nodeLoadEndnode_input_mode = true;
			snprintf(nodeLoadEndnode_str, 16, "%i", nodeLoadEndnode_input); // set the buffer to current Load End Node
		}
	}
	else // input mode
	{
		// Text box to input Load End Node
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputLoadEndNode", nodeLoadEndnode_str, IM_ARRAYSIZE(nodeLoadEndnode_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			nodeLoadEndnode_input = static_cast<int>(atoi(nodeLoadEndnode_str));
			// set the load End Node to input value
			if (nodeLoadEndnode_input > node_load_start)
			{
				// set the Load End Node to input value
				node_load_end = nodeLoadEndnode_input;
			}
			else
			{
				nodeLoadEndnode_input = static_cast<float>(node_load_end);
			}

		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			nodeLoadEndnode_input_mode = false;
		}
	}

	// Text for Load End Node
	ImGui::SameLine();
	ImGui::Text("Nodal Load End Node = %i", node_load_end);


}
