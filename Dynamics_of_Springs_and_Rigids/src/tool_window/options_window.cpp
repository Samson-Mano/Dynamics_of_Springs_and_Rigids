#include "options_window.h"

options_window::options_window()
{
	// Empty constructor
}

options_window::~options_window()
{
	// Empty destructor
}

void options_window::init()
{
	// Initialize the options
	is_show_constraint = true;
	is_show_loads = true;
	is_show_loadvalues = true;
	is_show_modelnodes = true;
	is_show_modelelements = true;
	is_show_window = false;
}

void options_window::render_window()
{
	if (is_show_window == false)
		return;

	// Create a new ImGui options window
	ImGui::Begin("View Options");

	// Add 4 checkboxes
	ImGui::Checkbox("Show Constraints", &is_show_constraint);
	ImGui::Checkbox("Show Loads", &is_show_loads);
	ImGui::Checkbox("Show Load values", &is_show_loadvalues);
	ImGui::Checkbox("Show Nodes", &is_show_modelnodes);
	ImGui::Checkbox("Show Elements", &is_show_modelelements);



	ImGui::Spacing();
	ImGui::Spacing();

	// Add a "Close" button
	if (ImGui::Button("Close"))
	{
		is_show_window = false;
	}

	ImGui::End();
}