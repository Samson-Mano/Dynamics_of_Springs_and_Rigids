#include "arcball_transformation.h"

arcball_transformation::arcball_transformation()
{
	// Empty constructor
}

arcball_transformation::~arcball_transformation()
{
	// Empty destructor
}

void arcball_transformation::OnMouseDown(glm::vec2 mousePt)
{
	this->startVector = ProjectToSphere(mousePt, 2.0f);
}

void arcball_transformation::OnMouseMove(glm::vec2 mousePt)
{
    endVector = ProjectToSphere(mousePt, 2.0f);

    // Compute the rotation quaternion from the start and end vectors
    glm::vec4 q;
    float dot = glm::dot(startVector, endVector);
    if (dot > 0.99999f)
    {
        // Vectors are almost parallel, no rotation is needed
        q = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f); // Identity quaternion
    }
    else if (dot < -0.99999f)
    {
        // Vectors are almost opposite, rotate around any axis perpendicular to startVector
        glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), startVector));
        q = Quaternion.FromAxisAngle(axis, (float)Math.PI);
        q = glm::(glm::pi<float>(), axis);
    }
    else
    {
        // Compute the rotation axis and angle
        glm::vec3 axis = glm::normalize(glm::cross(startVector, endVector));
        float angle = glm::acos(dot);
        q = Quaternion.FromAxisAngle(axis, angle);
    }

    // Update the rotation quaternion
    rotationQuaternion = glm::normalize(rotationQuaternion * q);

    // Set the end vector as the start vector for the next mouse move
    startVector = endVector;
}

glm::vec3 arcball_transformation::ProjectToSphere(glm::vec2 point, float radius)
{


}

void arcball_transformation::setDefault(const int& viewType)
{
}

glm::vec4 arcball_transformation::getRotationMatrix()
{
	return glm::vec4();
}
