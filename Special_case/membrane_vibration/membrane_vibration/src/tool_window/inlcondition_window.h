#pragma once
#include <iostream>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"

class inlcondition_window
{
public:
	bool is_show_window = false;
	bool execute_apply_displ = false;
	bool execute_remove_displ = false;
	bool execute_apply_velo = false;
	bool execute_remove_velo = false;

	// Initial Condition values
	// Displacement
	double inl_displacement = 10.0;
	int inl_displacement_start = 0;
	int inl_displacement_end = 10;
	int inl_displacement_type = 2;

	// Velocity
	double inl_velocity = 10.0;
	int inl_velocity_start = 20;
	int inl_velocity_end = 30;
	int inl_velocity_type = 2;

	inlcondition_window();
	~inlcondition_window();
	void init();
	void render_window();
private:

	void get_Initial_Displacement();
	void get_Initial_Displacement_Start_Node();
	void get_Initial_Displacement_End_Node();

	void get_Initial_Velocity();
	void get_Initial_Velocity_Start_Node();
	void get_Initial_Velocity_End_Node();

};