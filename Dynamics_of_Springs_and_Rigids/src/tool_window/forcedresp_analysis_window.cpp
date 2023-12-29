#include "forcedresp_analysis_window.h"

forcedresp_analysis_window::forcedresp_analysis_window()
{
	// Empty constructor
}

forcedresp_analysis_window::~forcedresp_analysis_window()
{
	// Empty destructor
}

void forcedresp_analysis_window::init()
{
	is_show_window = false;
	execute_forcedresp_analysis = false; // Main solver run event flag

	mode_result_str.clear(); // Remove the mode result list
	selected_modal_option1 = 0;
	selected_modal_option2 = 0;

	// Add to list
	dropdownlist_cstr.clear();

	// Displ Response
	std::string temp_str = "Magnitude Response";
	dropdownlist_cstr.push_back(temp_str.c_str());

	// X Response
	temp_str = "X Response";
	dropdownlist_cstr.push_back(temp_str.c_str());

	// Y Response
	temp_str = "Y Response";
	dropdownlist_cstr.push_back(temp_str.c_str());

	forcedresp_analysis_complete = false;
	frf_data.clear();

	// Selected node
	is_selection_changed = false;
	is_selected_count = false;
	selected_nodes.clear();
}


void forcedresp_analysis_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("Forced Response Analysis Solver");

	// Show Modal analysis results
//________________________________________________________________________________________
	ImGui::Text("Start Mode 1 Frequency = %.3f Hz", static_cast<float>(modal_first_frequency));
	ImGui::Text("End Mode %i Frequency = %.3f Hz", number_of_modes, static_cast<float>(modal_end_frequency));

	// Add the modal analysis result list box
	std::vector<const char*> items_cstr;
	for (const auto& item : mode_result_str)
	{
		items_cstr.push_back(item.c_str());
	}

	if (selected_modal_option1 > selected_modal_option2)
	{
		selected_modal_option2 = selected_modal_option1;
	}

	// Dropdown list 1 Mode range 1
	// Add the modal analysis result dropdown list
	const char* current_item1 = (selected_modal_option1 >= 0 && selected_modal_option1 < items_cstr.size()) ? items_cstr[selected_modal_option1] : "";

	if (ImGui::BeginCombo("Start Mode", current_item1))
	{
		for (int i = 0; i < items_cstr.size(); i++)
		{
			const bool is_selected = (selected_modal_option1 == i);
			if (ImGui::Selectable(items_cstr[i], is_selected))
			{
				selected_modal_option1 = i;
			}

			// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
			if (is_selected)
			{
				ImGui::SetItemDefaultFocus();
			}
		}

		ImGui::EndCombo();
	}

	// Dropdown list 2 Mode range 2
	// Add the modal analysis result dropdown list
	const char* current_item2 = (selected_modal_option2 >= 0 && selected_modal_option2 < items_cstr.size()) ? items_cstr[selected_modal_option2] : "";

	if (ImGui::BeginCombo("End Mode", current_item2))
	{
		for (int i = 0; i < items_cstr.size(); i++)
		{
			const bool is_selected = (selected_modal_option2 == i);
			if (ImGui::Selectable(items_cstr[i], is_selected))
			{
				selected_modal_option2 = i;
			}

			// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
			if (is_selected)
			{
				ImGui::SetItemDefaultFocus();
			}
		}

		ImGui::EndCombo();
	}

	ImGui::Text("Mode range %i to %i", (selected_modal_option1 + 1), (selected_modal_option2 + 1));

	//_________________________________________________________________________________________
	// Get the start frequency input

	get_frequency_start_input();

	//_________________________________________________________________________________________
	// Get the end frequency input

	get_frequency_end_input();

	//_________________________________________________________________________________________
	// Get the frequency interval

	get_frequency_interval();

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
	// Apply
	// Add a Forced Response Analysis button
	if (ImGui::Button("Forced Response Analysis"))
	{
		execute_forcedresp_analysis = true;
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

		execute_forcedresp_analysis = false; // Main solver run event flag

		is_show_window = false;
	}

	//__________________________________________________________________________________________
	ImGui::Spacing();

	// Analysis Results

	if (forcedresp_analysis_complete == true)
	{
		if (frf_data.empty() == false)
		{
			// Dropdown list (Response Option)
			// Add the Response options to the dropdown list
			const char* current_item3 = (selected_response_option >= 0 && selected_response_option < dropdownlist_cstr.size()) ? dropdownlist_cstr[selected_response_option] : "";

			if (ImGui::BeginCombo("Response", current_item3))
			{
				for (int i = 0; i < dropdownlist_cstr.size(); i++)
				{
					const bool is_selected = (selected_response_option == i);
					if (ImGui::Selectable(dropdownlist_cstr[i], is_selected))
					{
						selected_response_option = i;
					}

					// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
					if (is_selected)
					{
						ImGui::SetItemDefaultFocus();
					}
				}

				ImGui::EndCombo();
			}

			//__________________________________________________________________________________________

			if (selected_response_option == 0)
			{
				// Magnitude
				ImGui::Text("Contour Bar - Angular Acceleration");

				ImPlot::CreateContext();


				if (ImPlot::BeginPlot("Frequency vs. Magnitude Amplitude", ImVec2(-1, 0)))
				{

					ImPlot::SetupAxis(ImAxis_X1, "Frequency (Hz)");
					ImPlot::SetupAxis(ImAxis_Y1, "Magnitude Amplitude");

					ImPlot::SetupAxisLimits(ImAxis_X1, frf_chart_setting.chart_x_min, frf_chart_setting.chart_x_max); // Set X-axis limits
					ImPlot::SetupAxisLimits(ImAxis_Y1, frf_chart_setting.chart_y_min, frf_chart_setting.chart_y_max); // Set Y-axis limits


					for (int j = 0; j < static_cast<int>(frf_data.size()); j++)
					{
						std::string plotLabel = "Node " + std::to_string(frf_data[j].node_id);

						// Plot the X-Y data
						ImPlot::PlotLine(plotLabel.c_str(), frf_data[j].frequency_values.data(), frf_data[j].displ_magnitude.data(),
							frf_chart_setting.data_pt_count);
					}
					

					// End the plot
					ImPlot::EndPlot();
				}

				ImPlot::DestroyContext();



			}
			else if (selected_response_option == 1)
			{
				// X Response


			}
			else if (selected_response_option == 2)
			{
				// Y Response


			}



		}

	}

	//__________________________________________________________________________________________

	ImGui::End();

}


void forcedresp_analysis_window::add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right)
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


void forcedresp_analysis_window::get_frequency_start_input()
{
	// Input box to give input via text
	static bool freqstart_input_mode = false;
	static char freqstart_str[16] = ""; // buffer to store input frequency start string
	static float freqstart_input = static_cast<float>(start_frequency); // buffer to store input frequency start

	// Button to switch to input mode
	if (!freqstart_input_mode)
	{
		if (ImGui::Button("Input Start Frequency"))
		{
			freqstart_input_mode = true;
			snprintf(freqstart_str, 16, "%.3f", freqstart_input); // set the buffer to current frequency start 
		}
	}
	else // input mode
	{
		// Text box to input frequency start
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##FrequencyStart", freqstart_str, IM_ARRAYSIZE(freqstart_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			freqstart_input = static_cast<float>(atof(freqstart_str));
			// set the frequency start to input value
			start_frequency = freqstart_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			freqstart_input_mode = false;
		}
	}

	// Text for start frequency
	ImGui::SameLine();
	ImGui::Text("Start Frequency = %.3f", freqstart_input);


}


void forcedresp_analysis_window::get_frequency_end_input()
{
	// Input box to give input via text
	static bool freqend_input_mode = false;
	static char freqend_str[16] = ""; // buffer to store input load string
	static float freqend_input = static_cast<float>(end_frequency); // buffer to store input End Frequency

	// Button to switch to input mode
	if (!freqend_input_mode)
	{
		if (ImGui::Button("Input End Frequency"))
		{
			freqend_input_mode = true;
			snprintf(freqend_str, 16, "%.3f", freqend_input); // set the buffer to current End Frequency
		}
	}
	else // input mode
	{
		// Text box to input frequency end
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##FrequencyEnd", freqend_str, IM_ARRAYSIZE(freqend_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			freqend_input = static_cast<float>(atof(freqend_str));
			// set the frequency end to input value
			if (freqend_input > start_frequency)
			{
				end_frequency = freqend_input;
			}
			else
			{
				freqend_input = static_cast<float>(end_frequency);
			}
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			freqend_input_mode = false;
		}
	}

	// Text for end frequency
	ImGui::SameLine();
	ImGui::Text("End Frequency = %.3f", freqend_input);

}


void forcedresp_analysis_window::get_frequency_interval()
{
	// Text for Frequency interval
	//_________________________________________________________________________________________
	// Input box to give input via text
	static bool freqinterval_input_mode = false;
	static char freqinterval_str[16] = ""; // buffer to store input load string
	static float freqinterval_input = static_cast<float>(frequency_interval); // buffer to store input Frequency Interval

	// Button to switch to input mode
	if (!freqinterval_input_mode)
	{
		if (ImGui::Button("Input Frequency Interval"))
		{
			freqinterval_input_mode = true;
			snprintf(freqinterval_str, 16, "%.3f", freqinterval_input); // set the buffer to current Frequency Interval
		}
	}
	else // input mode
	{
		// Text box to input Frequency Interval
		ImGui::SetNextItemWidth(60.0f);
		if (ImGui::InputText("##InputFreqInterval", freqinterval_str, IM_ARRAYSIZE(freqinterval_str), ImGuiInputTextFlags_CharsDecimal))
		{
			// convert the input string to int
			freqinterval_input = static_cast<float>(atof(freqinterval_str));
			// set the load start time to input value
			frequency_interval = freqinterval_input;
		}

		// Button to switch back to slider mode
		ImGui::SameLine();
		if (ImGui::Button("OK"))
		{
			freqinterval_input_mode = false;
		}
	}


	// Text for Frequency Interval
	ImGui::SameLine();
	ImGui::Text("Frequency Interval = %.3f", frequency_interval);
	//_________________________________________________________________________________________

}