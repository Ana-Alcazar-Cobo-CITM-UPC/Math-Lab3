#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <fstream>
using namespace Eigen;
using namespace std;

#define EPSILON 1e-6
#define PI 2*acos(0.0)

Matrix3d EulerAxisAndAngle_ToRotationMatrix(const Vector3d& axis, double angle) {
	Vector3d normalizedAxis = axis.normalized();
	AngleAxisd angleAxis = AngleAxisd(angle, normalizedAxis);
	return angleAxis.toRotationMatrix();
}

//Function to perform quaternion product
Quaterniond MultiplyQuaternion(const Quaterniond& quaternion1, const Quaterniond& quaternion2)
{
	return quaternion1 * quaternion2;
}

Vector3d RotateVectorWithQuaternion(const Vector3d& vector, const Quaterniond& quaternion) {
	Quaterniond vectorQuaternion(0.0, vector.x(), vector.y(), vector.z());

	//q′= q⋅vq⋅q−1
	Quaterniond rotatedQuaternion = MultiplyQuaternion(quaternion, vectorQuaternion);
	rotatedQuaternion = MultiplyQuaternion(rotatedQuaternion, quaternion.inverse());
	return rotatedQuaternion.vec();
}

Matrix3d RotationMatrix_FromEulerAngles(double roll, double pitch, double yaw) {
	Matrix3d rotationMatrix(AngleAxisd(yaw, Vector3d::UnitZ()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(roll, Vector3d::UnitX()));
	return rotationMatrix;
}

Vector3d EulerAngles_FromRotationMatrix(const Matrix3d& rotationMatrix) {
	double roll;
	double pitch;
	double yaw;

	// Handle the singularity case for θ = π/2 + k·π (pitch)
	if (abs(rotationMatrix(2, 0)) != 1) {
		pitch = -asin(rotationMatrix(2, 0));
		roll = atan2(rotationMatrix(2, 1) / cos(pitch), rotationMatrix(2, 2) / cos(pitch)); 
		yaw = atan2(rotationMatrix(1, 0) / cos(pitch), rotationMatrix(0, 0) / cos(pitch));
	}
	else {
		yaw = 0; 
		if (rotationMatrix(2, 0) == -1) {
			pitch = PI / 2;
			roll = yaw + atan2(rotationMatrix(0, 1), rotationMatrix(0, 2));
		}
		else {
			pitch = -PI / 2;
			roll = -yaw + atan2(-rotationMatrix(0, 1), -rotationMatrix(0, 2));
		}
	}

	return Vector3d(roll, pitch, yaw);
}

pair<Vector3d, double> PrincipalAxisAngle_FromRotationMatrix(const Matrix3d& rotationMatrix) {
	AngleAxisd angleAxis(rotationMatrix);
	return { angleAxis.axis(), angleAxis.angle() };
}

Quaterniond Quaternion_FromAxisAngle(const Vector3d& axis, double angle) {
	return Quaterniond(AngleAxisd(angle, axis.normalized()));
}

pair<Vector3d, double> PrincipalAxisAngle_FromQuaternion(const Quaterniond& quaternion) {
	AngleAxisd angleAxis(quaternion);
	return { angleAxis.axis(), angleAxis.angle() };
}

Vector3d RotationVector_FromAxisAngle(const Vector3d& axis, double angle) {
	return axis.normalized() * angle; 
}

pair<Vector3d, double> PrincipalAxisAngle_FromRotationVector(const Vector3d& rotationVector) {
	double angle = rotationVector.norm();
	if (angle < EPSILON) {
		return { Vector3d(1, 0, 0), 0.0 };
	}
	return { rotationVector.normalized(), angle };
}

void convertRotations(char inputType,
	const Matrix3d& R_in = Matrix3d::Identity(),
	const Vector3d& euler_in = Vector3d::Zero(),
	const Vector3d& axis_in = Vector3d::Zero(),
	double angle_in = 0.0,
	const Quaterniond& quat_in = Quaterniond(1, 0, 0, 0),
	const Vector3d& rotVec_in = Vector3d::Zero()) {
	// Common outputs
	Matrix3d R;
	Vector3d euler;
	Vector3d axis;
	double angle;
	Quaterniond q;
	Vector3d rotationVector;

	// Process input based on type
	switch (inputType) {
	case 'r': // Rotation matrix as input
		R = R_in;
		euler = EulerAngles_FromRotationMatrix(R);
		tie(axis, angle) = PrincipalAxisAngle_FromRotationMatrix(R);
		q = Quaternion_FromAxisAngle(axis, angle);
		rotationVector = RotationVector_FromAxisAngle(axis, angle);
		break;

	case 'e': // Euler angles as input
		euler = euler_in;
		R = RotationMatrix_FromEulerAngles(euler.x(), euler.y(), euler.z());
		tie(axis, angle) = PrincipalAxisAngle_FromRotationMatrix(R);
		q = Quaternion_FromAxisAngle(axis, angle);
		rotationVector = RotationVector_FromAxisAngle(axis, angle);
		break;

	case 'p': // Principal axis/angle as input
		axis = axis_in.normalized();
		angle = angle_in;
		R = EulerAxisAndAngle_ToRotationMatrix(axis, angle);
		euler = EulerAngles_FromRotationMatrix(R);
		q = Quaternion_FromAxisAngle(axis, angle);
		rotationVector = RotationVector_FromAxisAngle(axis, angle);
		break;

	case 'q': // Quaternion as input
		q = quat_in.normalized();
		R = q.toRotationMatrix();
		euler = EulerAngles_FromRotationMatrix(R);
		tie(axis, angle) = PrincipalAxisAngle_FromQuaternion(q);
		rotationVector = RotationVector_FromAxisAngle(axis, angle);
		break;

	case 'v': // Rotation vector as input
		tie(axis, angle) = PrincipalAxisAngle_FromRotationVector(rotVec_in);
		R = EulerAxisAndAngle_ToRotationMatrix(axis, angle);
		euler = EulerAngles_FromRotationMatrix(R);
		q = Quaternion_FromAxisAngle(axis, angle);
		rotationVector = rotVec_in;
		break;

	default:
		cerr << "Invalid input type. Use 'r', 'e', 'p', 'q', or 'v'." << endl;
		return;
	}
}

int main()
{

	printf("EJ 1\n");

	Vector3d axis = { 1,0,0 };
	Matrix3d resultMatrix = EulerAxisAndAngle_ToRotationMatrix(axis, 30);

	double determinantValue = resultMatrix.determinant();
	printf("\tDeterminant: %f\n", determinantValue);
	if (resultMatrix.inverse() == resultMatrix.transpose()) {
		printf("\tInverse==Transpose: YES\n");
	}
	else {
		printf("\tInverse==Transpose: NO\n");
	}


	Vector3d parallel = axis; // Vector parallel to the axis
	Vector3d perpendicular(0, 1, 0); // Vector perpendicular to the x-axis

	Vector3d parallel_rotated = resultMatrix * parallel;
	Vector3d perpendicular_rotated = resultMatrix * perpendicular;

	cout << "\tParallel vector: " << parallel.transpose() << endl;
	cout << "\tRotated parallel vector: " << parallel_rotated.transpose() << endl;
	cout << "\tPerpendicular vector: " << perpendicular.transpose() << endl;
	cout << "\tRotated perpendicular vector: " << perpendicular_rotated.transpose() << endl;

	printf("EJ 2\n");

	printf("EJ 3\n");
	srand(static_cast<unsigned int> (time(0)));


	int num_steps = 100;
	double start_angle = 0.0;
	double end_angle = 6.0 * PI;
	double step = (end_angle - start_angle) / (num_steps - 1);

	vector<double> angles;
	vector<double> traces;

	for (size_t i = 0; i < num_steps; i++)
	{
		double angle = start_angle + i * step; 
		angles.push_back(angle);

		double x = rand() % 100;
		double y = rand() % 100;
		double z = rand() % 100;
		Vector3d axis = { x,y,z };
		axis = axis.normalized();

		Matrix3d matrix = EulerAxisAndAngle_ToRotationMatrix(axis, angle);
		traces.push_back(matrix.trace());
	}

	ofstream file("trace_vs_angle.csv");
	if (file.is_open()) {
		file << "Angle,Trace\n";
		for (size_t i = 0; i < angles.size(); ++i) {
			file << angles[i] << "," << traces[i] << "\n";
		}
		file.close();
		cout << "Results written to trace_vs_angle.csv" << endl;
	}
	else {
		cerr << "Unable to open file for writing." << endl;
		return 1;
	}

	
	return 0;
}