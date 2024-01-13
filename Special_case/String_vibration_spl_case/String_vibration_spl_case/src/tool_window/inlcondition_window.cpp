#include "inlcondition_window.h"

inlcondition_window::inlcondition_window()
{
	// Empty Constructor
}

inlcondition_window::~inlcondition_window()
{
	// Empty Destructor
}

void inlcondition_window::init()
{
	is_show_window = false;
	execute_apply_displ = false;
	execute_remove_displ = false;
	execute_apply_velo = false;
	execute_remove_velo = false;
}

void inlcondition_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("Initial Condition");

	// Create Displacement Initial Condition
	if (ImGui::CollapsingHeader("Displacement Initial Condition", ImGuiTreeNodeFlags_DefaultOpen))
	{
		// Inputs
		// Text for Initial Displacement
		//_________________________________________________________________________________________
		
		get_Initial_Displacement();

		//_____________________________________________________________________________________________________________________________________________________________________
		// Create four radio buttons for interpolation options

		ImGui::RadioButton("Linear Displacement Interpolation", &inl_displacement_type, 0);
		ImGui::RadioButton("Cubic Bezier Displacement Interpolation", &inl_displacement_type, 1);
		ImGui::RadioButton("Sine Displacement Interpolation", &inl_displacement_type, 2);
		ImGui::RadioButton("Rectangular Displacement Interpolation", &inl_displacement_type, 3);
		ImGui::RadioButton("Single Node Displacement", &inl_displacement_type, 4);

		// Text for Initial Displacement At Node
		//_________________________________________________________________________________________
		
		get_Initial_Displacement_Start_Node();

		// Text for Initial Displacement End Node
		//_________________________________________________________________________________________
		
		if (inl_displacement_type != 4)
		{
			get_Initial_Displacement_End_Node();
		}

		//_____________________________________________________________________________________________________________________________________________________________________
		ImGui::Spacing();
		if (ImGui::Button("Displacement Add"))
		{
			execute_apply_displ = true;
		}
		ImGui::SameLine();
		// Delete all Initial Displacement button
		if (ImGui::Button("Displacement Delete All"))
		{
			execute_remove_displ = true;
		}
	}

	ImGui::Spacing();

	// Create Velocity Initial Condition
	if (ImGui::CollapsingHeader("Velocity Initial Condition", ImGuiTreeNodeFlags_DefaultOpen))
	{
		// Inputs
		// Text for Initial Velocity
		//_________________________________________________________________________________________
		
		get_Initial_Velocity();

		//_____________________________________________________________________________________________________________________________________________________________________

		// Create four radio buttons for interpolation options
		ImGui::RadioButton("Linear Velocity Interpolation", &inl_velocity_type, 0);
		ImGui::RadioButton("Cubic Bezier Velocity Interpolation", &inl_velocity_type, 1);
		ImGui::RadioButton("Sine Velocity Interpolation", &inl_velocity_type, 2);
		ImGui::RadioButton("Rectangular Velocity Interpolation", &inl_velocity_type, 3);
		ImGui::RadioButton("Single Node Velocity", &inl_velocity_type, 4);

		// Text for Initial Velocity At Node
		//_________________________________________________________________________________________
		
		get_Initial_Velocity_Start_Node();

		// Text for Initial Velocity End Node
		//_________________________________________________________________________________________
		
		if (inl_velocity_type != 4)
		{
			get_Initial_Velocity_End_Node();
		}

		//_____________________________________________________________________________________________________________________________________________________________________
		ImGui::Spacing();
		if (ImGui::Button("Velocity Add"))
		{
			execute_apply_velo = true;
		}
		ImGui::SameLine();
		// Delete all Initial Displacement button
		if (ImGui::Button("Velocity Delete All"))
		{
			execute_remove_velo = true;
		}
	}

	//_____________________________________________________________________________________________________________________________________________________________________

	ImGui::Spacing();
	ImGui::Spacing();

	// Add a "Close" button
	if (ImGui::Button("Close"))
	{
		is_show_window = false;
	}

	ImGui::End();

}



void inlcondition_window::get_Initial_Displacement()
{
	// Text for Initial Displacement
	//_________________________________________________________________________________________
	// Input box to give input via text
	static bool inlDisplacement_input_mode = false;
	static char inlDisplacement_str[16] = ""; // buffer to store input Initial Displacement string
	static float inlDisplacement_input = static_cast<float>(inl_displacement); // buffer to store input Initial Displacement

	// Button to switch to input mode
	if (!inlDisplacement_input_mode)
	{
		if (ImGui::Button("Initial Displacement"))
		{
			inlDisplacement_input_mode = true;
			snprintf(inlDisplacement_str, 16, "%.3f", inlDisplacement_input); // set the buffer to current Initial Displacement
		}
	}
	else // input mode
	{
		// Text box to input Initial Displacement
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputInitialDisplacement", inlDisplacement_str, IM_ARRAYSIZE(inlDisplacement_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to float
			inlDisplacement_input = static_cast<float>(atof(inlDisplacement_str));
			// set the Initial Displacement to input value
			inl_displacement = inlDisplacement_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			inlDisplacement_input_mode = false;
		}
	}

	// Text for Initial Displacement
	ImGui::SameLine();
	ImGui::Text("Initial Displacement = %.3f", inl_displacement);

}


void inlcondition_window::get_Initial_Displacement_Start_Node()
{
	// Text for Initial Displacement At Node
		//_________________________________________________________________________________________
		// Input box to give input via text
	static bool inlDisplacementStart_input_mode = false;
	static char inlDisplacementStart_str[16] = ""; // buffer to store input Initial Displacement At Node string
	static int inlDisplacementStart_input = static_cast<int>(inl_displacement_start); // buffer to store input Initial Displacement At Node

	// Button to switch to input mode
	if (!inlDisplacementStart_input_mode)
	{
		if (ImGui::Button("Initial Displacement Start Node"))
		{
			inlDisplacementStart_input_mode = true;
			snprintf(inlDisplacementStart_str, 16, "%i", inlDisplacementStart_input); // set the buffer to current Initial Displacement At Node
		}
	}
	else // input mode
	{
		// Text box to input Initial Displacement At Node
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputInitialDisplacementStartNode", inlDisplacementStart_str, IM_ARRAYSIZE(inlDisplacementStart_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			inlDisplacementStart_input = static_cast<int>(atoi(inlDisplacementStart_str));
			// set the Initial Displacement At Node to input value
			inl_displacement_start = inlDisplacementStart_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			inlDisplacementStart_input_mode = false;
		}
	}

	if (inl_displacement_start >= inl_displacement_end)
	{
		inl_displacement_end = inl_displacement_start + 1;
	}

	// Text for Initial Displacement At Node
	ImGui::SameLine();
	ImGui::Text("Initial Displacement Start Node = %i", inl_displacement_start);

}


void inlcondition_window::get_Initial_Displacement_End_Node()
{
	// Text for Initial Displacement End Node
	//_________________________________________________________________________________________
	// Input box to give input via text
	static bool inlDisplacementEnd_input_mode = false;
	static char inlDisplacementEnd_str[16] = ""; // buffer to store input Initial Displacement End Node string
	static int inlDisplacementEnd_input = static_cast<int>(inl_displacement_end); // buffer to store input Initial Displacement End Node

	// Button to switch to input mode
	if (!inlDisplacementEnd_input_mode)
	{
		if (ImGui::Button("Initial Displacement End Node"))
		{
			inlDisplacementEnd_input_mode = true;
			snprintf(inlDisplacementEnd_str, 16, "%i", inlDisplacementEnd_input); // set the buffer to current Initial Displacement End Node
		}
	}
	else // input mode
	{
		// Text box to input Initial Displacement End Node
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputInitialDisplacementEndNode", inlDisplacementEnd_str, IM_ARRAYSIZE(inlDisplacementEnd_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			inlDisplacementEnd_input = static_cast<int>(atoi(inlDisplacementEnd_str));
			// set the load End Node to input value
			if (inlDisplacementEnd_input > inl_displacement_start)
			{
				// set the Initial Displacement End Node to input value
				inl_displacement_end = inlDisplacementEnd_input;
			}
			else
			{
				inlDisplacementEnd_input = static_cast<float>(inl_displacement_end);
			}
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			inlDisplacementEnd_input_mode = false;
		}
	}

	// Text for Initial Displacement End Node
	ImGui::SameLine();
	ImGui::Text("Initial Displacement End Node = %i", inl_displacement_end);


}


void inlcondition_window::get_Initial_Velocity()
{
	// Text for Initial Velocity
		//_________________________________________________________________________________________
		// Input box to give input via text
	static bool inlVelocity_input_mode = false;
	static char inlVelocity_str[16] = ""; // buffer to store input Initial Velocity string
	static float inlVelocity_input = static_cast<float>(inl_velocity); // buffer to store input Initial Velocity

	// Button to switch to input mode
	if (!inlVelocity_input_mode)
	{
		if (ImGui::Button("Initial Velocity"))
		{
			inlVelocity_input_mode = true;
			snprintf(inlVelocity_str, 16, "%.3f", inlVelocity_input); // set the buffer to current Initial Velocity
		}
	}
	else // input mode
	{
		// Text box to input Initial Velocity
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputInitialVelocity", inlVelocity_str, IM_ARRAYSIZE(inlVelocity_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to float
			inlVelocity_input = static_cast<float>(atof(inlVelocity_str));
			// set the Initial Velocity to input value
			inl_velocity = inlVelocity_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			inlVelocity_input_mode = false;
		}
	}

	// Text for Initial Velocity
	ImGui::SameLine();
	ImGui::Text("Initial Velocity = %.3f", inl_velocity);

}



void inlcondition_window::get_Initial_Velocity_Start_Node()
{
	// Text for Initial Velocity At Node
		//_________________________________________________________________________________________
		// Input box to give input via text
	static bool inlVelocityStart_input_mode = false;
	static char inlVelocityStart_str[16] = ""; // buffer to store input Initial Velocity Start Node string
	static int inlVelocityStart_input = static_cast<int>(inl_velocity_start); // buffer to store input Initial Velocity Start Node

	// Button to switch to input mode
	if (!inlVelocityStart_input_mode)
	{
		if (ImGui::Button("Initial Velocity Start Node"))
		{
			inlVelocityStart_input_mode = true;
			snprintf(inlVelocityStart_str, 16, "%i", inlVelocityStart_input); // set the buffer to current Initial Velocity Start Node
		}
	}
	else // input mode
	{
		// Text box to input Initial Velocity Start Node
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputInitialVelocityAtNode", inlVelocityStart_str, IM_ARRAYSIZE(inlVelocityStart_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			inlVelocityStart_input = static_cast<int>(atoi(inlVelocityStart_str));
			// set the Initial Velocity Start Node to input value
			inl_velocity_start = inlVelocityStart_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			inlVelocityStart_input_mode = false;
		}
	}

	if (inl_velocity_start >= inl_velocity_end)
	{
		inl_velocity_end = inl_velocity_start + 1;
	}

	// Text for Initial Velocity At Node
	ImGui::SameLine();
	ImGui::Text("Initial Velocity At Node = %i", inl_velocity_start);


}


void inlcondition_window::get_Initial_Velocity_End_Node()
{
	// Text for Initial Velocity End Node
	//_________________________________________________________________________________________
	// Input box to give input via text
	static bool inlVelocityEnd_input_mode = false;
	static char inlVelocityEnd_str[16] = ""; // buffer to store input Initial Velocity End Node string
	static int inlVelocityEnd_input = static_cast<int>(inl_velocity_end); // buffer to store input Initial Velocity End Node

	// Button to switch to input mode
	if (!inlVelocityEnd_input_mode)
	{
		if (ImGui::Button("Initial Velocity End Node"))
		{
			inlVelocityEnd_input_mode = true;
			snprintf(inlVelocityEnd_str, 16, "%i", inlVelocityEnd_input); // set the buffer to current Initial Velocity End Node
		}
	}
	else // input mode
	{
		// Text box to input Initial Velocity End Node
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputInitialVelocityEndNode", inlVelocityEnd_str, IM_ARRAYSIZE(inlVelocityEnd_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			inlVelocityEnd_input = static_cast<int>(atoi(inlVelocityEnd_str));
			// set the load End Node to input value
			if (inlVelocityEnd_input > inl_velocity_start)
			{
				// set the Initial Velocity End Node to input value
				inl_velocity_end = inlVelocityEnd_input;
			}
			else
			{
				inlVelocityEnd_input = static_cast<float>(inl_velocity_end);
			}
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			inlVelocityEnd_input_mode = false;
		}
	}

	// Text for Initial Displacement End Node
	ImGui::SameLine();
	ImGui::Text("Initial Velocity End Node = %i", inl_velocity_end);

}


