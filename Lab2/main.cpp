#include <Eigen/Dense>

using namespace Eigen;

Matrix3d EulerAxisAndAngle_ToRotationMatrix(const Vector3d& axis, double angle) {
	Vector3d normalizedAxis = axis.normalized();
	AngleAxisd angleAxis = AngleAxisd(angle, normalizedAxis);
	return angleAxis.toRotationMatrix();
}

int main() {

	Matrix3d resultMatrix = EulerAxisAndAngle_ToRotationMatrix({0,1,0}, 30);
	double determinantValue = resultMatrix.determinant();
	printf("%f\n", determinantValue);
	if (resultMatrix.inverse() == resultMatrix.transpose()) {
		printf("YES\n");
	}

	return 0;
}