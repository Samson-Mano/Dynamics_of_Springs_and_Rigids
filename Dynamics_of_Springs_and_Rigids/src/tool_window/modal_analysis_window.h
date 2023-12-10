#pragma once
#include <iostream>
#include <vector>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/stb_implement.h"
#include "../geometry_store/geom_parameters.h"


class modal_analysis_window
{
public:
	bool is_show_window = false;
	bool execute_modal_analysis = false; // Main solver run event flag
	bool execute_modal_open = false; // Solver window execute opening event flag
	bool execute_modal_close = false; // Closing of solution window event flag

	// Modal analysis result list
	int selected_modal_option = 0;
	std::vector<std::string> mode_result_str;
	int selection_change_flag = 0;
	bool is_mode_selection_changed = false;

	// analysis results
	bool modal_analysis_complete = false;
	bool show_undeformed_model = true; // show undeformed model 
	bool show_result_text_values = true; // show the result text values

	// Animation control
	bool animate_play = true;
	bool animate_pause = false;
	double deformation_scale_max = 10.0;
	double animation_speed = 1.0;

	// Time step control
	double time_interval_atrun = 0.0; // Value of time interval used in the pulse response 
	int time_step_count = 0;
	int time_step = 0;


	modal_analysis_window();
	~modal_analysis_window();
	void init(); // initialize
	void render_window();
private:
	Stopwatch_events stopwatch;

};