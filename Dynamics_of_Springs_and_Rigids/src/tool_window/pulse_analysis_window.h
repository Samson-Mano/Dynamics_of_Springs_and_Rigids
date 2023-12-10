#pragma once
#include <iostream>
#include <vector>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/stb_implement.h"
#include "../geometry_store/geom_parameters.h"


class pulse_analysis_window
{
public:
	bool is_show_window = false;
	bool execute_pulse_analysis = false; // Main solver run event flag
	bool execute_pulse_open = false; // Solver window execute opening event flag
	bool execute_pulse_close = false; // Closing of solution window event flag



	// analysis results
	bool pulse_analysis_complete = false;
	bool show_undeformed_model = true; // show undeformed model 


	pulse_analysis_window();
	~pulse_analysis_window();
	void init();// initialize
	void render_window();
private:
	Stopwatch_events stopwatch;

};
