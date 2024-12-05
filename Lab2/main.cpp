#include <Eigen/Dense>
#include <math.h>
using namespace Eigen;

#define PI 2*acos(0.0)

Matrix3d EulerAxisAndAngle_ToRotationMatrix(const Vector3d& axis, double angle) {
	Vector3d normalizedAxis = axis.normalized();
	AngleAxisd angleAxis = AngleAxisd(angle, normalizedAxis);
	return angleAxis.toRotationMatrix();
}

//Function to perform quaternion product
Quaterniond QuaternionMultiplication(const Quaterniond& q1, const Quaterniond& q2)
{
	return q1 * q2;
}

int main()
{

	Matrix3d resultMatrix = EulerAxisAndAngle_ToRotationMatrix({0,1,0}, 30);
	double determinantValue = resultMatrix.determinant();
	printf("%f\n", determinantValue);
	if (resultMatrix.inverse() == resultMatrix.transpose()) {
		printf("YES\n");
	}

	srand(static_cast<unsigned int> (time(0)));
	for (size_t i = 0; i <= 100; i++)
	{
		int x = rand() % 100;
		int y = rand() % 100;
		int z = rand() % 100;

		Vector3d axis = { x,y,z };
		axis = axis.normalized();
		Matrix3d matrix = EulerAxisAndAngle_ToRotationMatrix({ x,y,z }, 6 * PI * i / 100);
	}

	
	return 0;
}