#include "new_model_window.h"

new_model_window::new_model_window()
{
	// Empty constructor
}

new_model_window::~new_model_window()
{
	// Empty destructor
}

void new_model_window::init()
{
	is_show_window = false;
	execute_create_model = false;

}

void new_model_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("New Model");

	//_____________________________________________________________________________________________________________________________________________________________________
	// Create three radio buttons for model options

	ImGui::RadioButton("60 Segment quad mesh", &option_model_type, 0);
	ImGui::RadioButton("60 Segment quad mesh SPL1",&option_model_type,1);
	ImGui::RadioButton("60 Segment tri mesh", &option_model_type, 2);
	ImGui::RadioButton("60 Segment tri mesh SPL1", &option_model_type, 3);
	ImGui::RadioButton("60 Segment tri mesh SPL2", &option_model_type, 4);
	ImGui::RadioButton("120 Segment quad mesh", &option_model_type, 5);
	ImGui::RadioButton("180 Segment quad mesh", &option_model_type, 6);
	ImGui::RadioButton("240 Segment quad mesh", &option_model_type, 7);
	ImGui::RadioButton("360 Segment quad mesh", &option_model_type, 8);
	ImGui::RadioButton("720 Segment quad mesh", &option_model_type, 9);
	ImGui::RadioButton("1440 Segment quad mesh", &option_model_type, 10);


	//_____________________________________________________________________________________________________________________________________________________________________
	// Text box for input
	static char temp_str[1024 * 16] = "Tension, 6000\nDensity, 24";

	//ImGui::InputTextMultiline("##InputText",
	//	temp_str,
	//	IM_ARRAYSIZE(temp_str),
	//	ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 16),
	//	ImGuiInputTextFlags_AllowTabInput | ImGuiInputTextFlags_EnterReturnsTrue);


	// Parse the input string into a vector of strings
	input_data.clear();
	std::istringstream iss(temp_str);
	std::string line;

	while (std::getline(iss, line, '\n')) 
	{
		input_data.push_back(line);
	}


	//_____________________________________________________________________________________________________________________________________________________________________
	ImGui::Spacing();
	if (ImGui::Button("Create Model"))
	{
		execute_create_model = true;
	}

	//__________________________________________________________________________________________
	ImGui::Spacing();

	// Close button
	if (ImGui::Button("Close"))
	{
		is_show_window = false;
		execute_create_model = false;
	}

	//__________________________________________________________________________________________



	ImGui::End();




}
