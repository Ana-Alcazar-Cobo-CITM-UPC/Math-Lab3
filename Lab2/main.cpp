#include <Eigen/Dense>

using namespace Eigen;

Matrix3d RotationMatrixFromEulerAxisAndAngle(const Vector3d& axis, double angle) {
	Vector3d normalizedAxis = axis.normalized();
	AngleAxisd angleAxis = AngleAxisd(angle, axis);
	angleAxis.toRotationMatrix();
}

int main() {
	return 0;
}