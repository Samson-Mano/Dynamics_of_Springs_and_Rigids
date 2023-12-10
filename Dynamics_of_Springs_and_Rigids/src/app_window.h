#pragma once
#include <iostream>
#include <fstream>
//____ OpenGL dependencies
#include <GL/glew.h>
#include <GLFW/glfw3.h>
//____ ImGUI dependencies
#include "ImGui/imgui.h"
#include "ImGui/imgui_impl_glfw.h"
#include "ImGui/imgui_impl_opengl3.h"
#include "ImGui/stb_implement.h"
//____ Geometry store
#include "geometry_store/geom_store.h"
//____ Mouse event handler
#include "events_handler/mouse_event_handler.h"
//---- File event handler
#include "events_handler/file_events.h"
//____ Tool Window
#include "tool_window/node_constraint_window.h"
#include "tool_window/node_load_window.h"
#include "tool_window/inlcondition_window.h"
#include "tool_window/pointmass_window.h"
#include "tool_window/options_window.h"
#include "tool_window/element_prop_window.h"
#include "tool_window/modal_analysis_window.h"
#include "tool_window/pulse_analysis_window.h"
#include "tool_window/forced_analysis_window.h"


class app_window
{
public:
	GLFWwindow* window = nullptr;
	ImFont* imgui_font = nullptr;

	// Variable to control the windows mouse events
	mouse_event_handler mouse_Handler;
	// Variable to control the file menu events
	file_events file_menu;

	bool is_glwindow_success = false;
	static int window_width;
	static int window_height;
	static bool isWindowSizeChanging;

	// main geometry variable
	geom_store geom;

	// Tool window variable
	node_constraint_window nd_cnst_window;
	node_load_window nd_load_window;
	pointmass_window nd_ptmass_window;
	inlcondition_window nd_inlcond_window;
	options_window op_window;
	element_prop_window elm_prop_window;

	// Analysis window
	modal_analysis_window modal_solver_window;
	pulse_analysis_window pulse_solver_window;
	forced_analysis_window forced_solver_window;


	app_window();
	~app_window();

	// Functions
	void init();
	void fini();
	void app_render();
	void menu_events();
	static void framebufferSizeCallback(GLFWwindow* window, int window_width, int window_height);
	void GLFWwindow_set_icon(GLFWwindow* window);
private:
};