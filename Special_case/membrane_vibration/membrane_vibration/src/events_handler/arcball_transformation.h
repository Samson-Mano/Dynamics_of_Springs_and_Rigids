#pragma once
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <unordered_map>
#include "../geometry_store/geom_parameters.h"

class arcball_transformation
{
public:
	arcball_transformation();
	~arcball_transformation();
	void OnMouseDown(glm::vec2 mousePt);
	void OnMouseMove(glm::vec2 mousePt);
	glm::vec3 ProjectToSphere(glm::vec2 point, float radius);
	void setDefault(const int& viewType);
	glm::vec4 getRotationMatrix();

private:
	glm::vec3 startVector = glm::vec3(0); // Start vector
	glm::vec3 endVector = glm::vec3(0); // End vector
	glm::vec4 rotationQuaternion = glm::vec4(0); // Rotation quaternion
	glm::mat4 rotationMatrix = glm::mat4(1.0f); // Rotation matrix

};