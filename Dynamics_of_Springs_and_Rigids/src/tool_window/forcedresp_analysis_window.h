#pragma once
#include <iostream>
#include <vector>
#include "../ImGui/imgui.h"
#include "../ImGui/implot.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/stb_implement.h"
#include "../geometry_store/geom_parameters.h"


class forcedresp_analysis_window
{
public:
	bool is_show_window = false;
	bool execute_forcedresp_analysis = false; // Main solver run event flag
	bool execute_forcedresp_open = false; // Solver window execute opening event flag
	bool execute_forcedresp_close = false; // Closing of solution window event flag

	double damping_ratio = 0.01; // Damping ratio

	// Modal analysis Results
	double modal_first_frequency = 0.0;
	double modal_end_frequency = 0.0;
	int number_of_modes = 0;

	std::vector<std::string> mode_result_str;
	int selected_modal_option1 = 0;
	int selected_modal_option2 = 0;

	// Frequency range input
	double start_frequency = 0.0; // Frequency response Start Frequency
	double end_frequency = 0.0; // Frequency respons End Frequency
	double frequency_interval = 0.01; // Frequency Interval

	// Forced response analysis results
	bool forcedresp_analysis_complete = false;

	// Selected node
	bool is_selection_changed = false;
	bool is_selected_count = false;
	std::vector<int> selected_nodes;

	// Frequency Response Data
	std::vector<frequency_reponse_data> frf_data;
	chart_setting_data frf_chart_setting;

	forcedresp_analysis_window();
	~forcedresp_analysis_window();
	void init();// initialize
	void render_window();
	void add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right);
private:
	std::vector<const char*> dropdownlist_cstr;
	int selected_response_option = 0;

	void get_frequency_start_input();
	void get_frequency_end_input();
	void get_frequency_interval();

};
