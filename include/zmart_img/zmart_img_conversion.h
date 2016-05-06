#ifndef ZMART_IMG_CONVERSION_H_
#define ZMART_IMG_CONVERSION_H_

#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

Quaterniond euler2quaternion(Vector3d euler);
Matrix3d quaternion2mat(Quaterniond q);
Vector3d mat2euler(Matrix3d m);
Quanterniond mat2quanternion(Matric3d m);
Matrix3d euler2mat(Vector3d euler);
Vector3d quaternion2euler(Quaterniond q);







#endif
