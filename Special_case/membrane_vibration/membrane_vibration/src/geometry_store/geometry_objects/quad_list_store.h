#pragma once
#include "point_list_store.h"

struct quad_store
{
	// store the individual point
	int quad_id = 0;

	// Location
	glm::vec3 quadpt1_loc = glm::vec3(0);
	glm::vec3 quadpt2_loc = glm::vec3(0);
	glm::vec3 quadpt3_loc = glm::vec3(0);
	glm::vec3 quadpt4_loc = glm::vec3(0);

	// color
	glm::vec3 quadpt1_color = glm::vec3(0);
	glm::vec3 quadpt2_color = glm::vec3(0);
	glm::vec3 quadpt3_color = glm::vec3(0);
	glm::vec3 quadpt4_color = glm::vec3(0);

};


class quad_list_store
{
public:
	geom_parameters* geom_param_ptr = nullptr;
	unsigned int quad_count = 0;
	std::vector<quad_store> quadMap;


	quad_list_store();
	~quad_list_store();

	void init(geom_parameters* geom_param_ptr);
	void add_quad(int& quad_id, const glm::vec3& quadpt1_loc, const glm::vec3& quadpt2_loc, 
		const glm::vec3& quadpt3_loc, const glm::vec3& quadpt4_loc,
		glm::vec3& quadpt1_color, glm::vec3& quadpt2_color, 
		glm::vec3& quadpt3_color, glm::vec3& quadpt4_color);
	void set_buffer();
	void paint_quadrilaterals();
	void clear_quadrilaterals();
	void update_opengl_uniforms(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);
private:
	gBuffers quad_buffer;
	Shader quad_shader;

	void get_quad_buffer(quad_store& quad, float* quad_vertices, unsigned int& quad_v_index, unsigned int* quad_vertex_indices, unsigned int& quad_i_index);


};
