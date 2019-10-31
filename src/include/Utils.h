#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <Eigen/Geometry>
using namespace Eigen;

// Compute euclidian 2D distance between 2 Vector3d
float distance(Vector3d posT,Vector3d posRH);

// Compute the angle of a body according to his Quarterniond (rotation around z only)
double thetaFct(Quaterniond q);

// Put a value to zero if it's too low (<1e-4)
double nullFunction(double val);

#endif // UTILS_H_INCLUDED