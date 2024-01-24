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
    glm::quat q;
    float dot = glm::dot(startVector, endVector);

    if (dot > 0.99999f)
    {
        // Vectors are almost parallel, no rotation is needed
        q = glm::quat(); // Identity quaternion
    }
    else if (dot < -0.99999f)
    {
        // Vectors are almost opposite, rotate around any axis perpendicular to startVector
        glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), startVector));
        q = glm::angleAxis(glm::pi<float>(), axis);
    }
    else
    {
        // Compute the rotation axis and angle
        glm::vec3 axis = glm::normalize(glm::cross(startVector, endVector));
        float angle = glm::acos(dot);
        q = glm::angleAxis(angle, axis);
    }

    // Update the rotation quaternion
    this->rotationQuaternion = glm::normalize(rotationQuaternion * q);

    // Set the end vector as the start vector for the next mouse move
    startVector = endVector;
}

glm::vec3 arcball_transformation::ProjectToSphere(glm::vec2 point, float radius)
{
    // Scale point to range [-1,1]
    float x = point.x / radius;
    float y = point.y / radius;

    // Compute square of the length of the vector from this point to the center
    float d = (x * x) + (y * y);

    if (d > 1.0f)
    {
        // Point is outside the sphere, project onto the sphere surface
        float s = 1.0f / (float)std::sqrt(d);
        return glm::vec3(x * s, y * s, 0.0f);
    }
    else
    {
        // Point is inside the sphere, compute z coordinate
        float z = (float)std::sqrt(1.0f - d);
        return glm::vec3(x, y, z);
    }
}

void arcball_transformation::setDefault(const int& viewType)
{
    if (viewType == 1)
    {
        // Isometric 1
        this->rotationQuaternion = glm::quat(0.5000000418651581f, 0.9330127233867831f, 0.33715310920426594f, -0.23570224007473873f);
    }
    else if (viewType == 2)
    {
        // Top view
        this->rotationQuaternion = glm::quat( 0.707107f,0.707107f, 0.0f, 0.0f);
           
    }
    else if (viewType == 3)
    {
        // Bottom view
        this->rotationQuaternion = glm::quat(-0.707107f,0.707107f, 0.0f, 0.0f );
           
    }
    else if (viewType == 4)
    {
        // Front view
        this->rotationQuaternion = glm::quat(1.0f,0.0f, 0.0f, 0.0f);
           
    }
    else if (viewType == 5)
    {
        // Back view
        this->rotationQuaternion = glm::quat(0.0f,0.0f, 1.0f, 0.0f);
           
    }
    else if (viewType == 6)
    {
        // Right view
        this->rotationQuaternion = glm::quat(0.5f,0.5f, 0.5f, -0.5f);
           
    }
    else if (viewType == 7)
    {
        // Left view
        this->rotationQuaternion = glm::quat(0.5f,0.5f, -0.5f, 0.5f);
    }

   //     Isometric view 1: ( 0.588156f, 0.377964f, 0.661438f, -0.225403f)
   //     Isometric view 2 : (0.377964f, 0.661438f, 0.225403f, -0.588156f)
   //     Isometric view 3 : (0.661438f, 0.225403f, -0.588156f, -0.377964f)
   //     Isometric view 4 : (0.588156f, -0.377964f, -0.661438f, 0.225403f)
   //     Isometric view 5 : (0.377964f, -0.661438f, -0.225403f, 0.588156f)
   //     Isometric view 6 : (0.661438f, -0.225403f, 0.588156f, 0.377964f)

}

glm::mat4 arcball_transformation::getRotationMatrix()
{
    return glm::mat4_cast(this->rotationQuaternion);
}
