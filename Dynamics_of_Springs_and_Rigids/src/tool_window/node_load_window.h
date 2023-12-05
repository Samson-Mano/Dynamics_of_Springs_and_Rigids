#pragma once
#include <iostream>
#include <cstring>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/stb_image.h"
#include "../ImGui/stb_implement.h"
#include "../geometry_store/geom_parameters.h"


struct load_image_data
{
	int image_width = 0;
	int image_height = 0;
	unsigned int image_texture_ID = 0;
	bool is_loaded = false;
};


class node_load_window
{
public:
	bool is_show_window = false;
	bool is_selection_changed = false;
	bool is_selected_count = false;

	bool apply_nodal_load = false;// Apply nodal load
	bool delete_nodal_load = false; // Delete nodal load
	std::vector<int> selected_nodes;

	double load_amplitude = 100.0; // load value
	double load_start_time = 0.2; // load start time
	double load_end_time = 0.6; // load end time
	double load_angle = 90.0; // load angle

	node_load_window();
	~node_load_window();
	void init(); // initialize bind images
	void render_window();
	void add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right);
private:
	load_image_data load_image;

	void get_load_value_input();
	void get_load_starttime_input();
	void get_load_endtime_input();
	void get_load_angle_input();
	void draw_load();
	void LoadTextureFromFile(const char* filename, load_image_data& load_image);
	bool get_image_min_max_coord(ImVec2& window_pos, ImVec2& window_size, ImVec2& img_pos_top_left,
		ImVec2& img_pos_top_right, ImVec2& img_pos_bot_right, ImVec2& img_pos_bot_left, double& orientation);

};
