#pragma once
#include <iostream>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"


class node_load_window
{
public:
	bool is_show_window = false;
	bool execute_apply_load = false;
	bool execute_remove_load = false;
	
	// Load values
	double load_amplitude = 10.0;
	double load_start_time = 0.2; // load start time
	double load_end_time = 0.6; // load end time

	int node_load_start = 10;
	int node_load_end = 40;
	int node_load_type = 2;

	node_load_window();
	~node_load_window();
	void init(); // initialize bind images
	void render_window();
private:

	void get_load_value_input();
	void get_load_starttime_input();
	void get_load_endtime_input();

	void get_load_Start_Node();
	void get_load_End_Node();

};
