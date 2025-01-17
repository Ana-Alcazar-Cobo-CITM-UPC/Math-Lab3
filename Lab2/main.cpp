#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <fstream>
using namespace Eigen;
using namespace std;

#define EPSILON 1e-6

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define PI 3.14159265358979323846

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
	if (std::abs(rotationMatrix(2, 0)) < 1.0 - EPSILON) {
		pitch = -asin(rotationMatrix(2, 0));
		double cosPitch = cos(pitch);

		yaw = atan2(rotationMatrix(1, 0) / cosPitch, rotationMatrix(0, 0) / cosPitch);
		roll = atan2(rotationMatrix(2, 1) / cosPitch, rotationMatrix(2, 2) / cosPitch);
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

void Convert(char inputType,
	Matrix3d& R_in,
	Vector3d& euler_in ,
	Vector3d& axis_in,
	double angle_in,
	Quaterniond& quat_in,
	Vector3d& rotVec_in) {
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

	R_in = R;
	euler_in = euler;
	axis_in = axis;
	angle_in = angle;
	quat_in = q;
	rotVec_in = rotationVector;

}

int main()
{
	Quaterniond q;
	Vector3d axis = { 1,1,0 };
	q = Quaternion_FromAxisAngle(axis, 30 * DEGTORAD);
	cout << "\tQuat: " << q << endl << endl;


	/*printf("EJ 1\n");

	Vector3d axis = { 0,1,0 };
	float angle = PI / 4;

	Matrix3d resultMatrix = EulerAxisAndAngle_ToRotationMatrix(axis, angle);
	cout << "\tAxis: " << axis.transpose() << endl;
	cout << "\tAngle: " << angle << endl << endl;
	cout << "\tResult RotationMatrix: " << endl << resultMatrix << endl << endl;

	double determinantValue = resultMatrix.determinant();
	printf("\tDeterminant: %f\n", determinantValue);
	if (resultMatrix.inverse() == resultMatrix.transpose()) {
		printf("\tInverse==Transpose: YES\n");
	}
	else {
		printf("\tInverse==Transpose: NO\n");
	}

	Vector3d parallel = axis;
	Vector3d perpendicular(1, 0, 0);

	Vector3d parallel_rotated = resultMatrix * parallel;
	Vector3d perpendicular_rotated = resultMatrix * perpendicular;

	cout << "\tParallel vector: " << parallel.transpose() << endl;
	cout << "\tRotated parallel vector: " << parallel_rotated.transpose() << endl;
	cout << "\tPerpendicular vector: " << perpendicular.transpose() << endl;
	cout << "\tRotated perpendicular vector: " << perpendicular_rotated.transpose() << endl << endl;

	printf("EJ 2\n");
	Vector3d vector = { 3,2,1 };
	Quaterniond quaternion = { 0.5f,0.5f,0.5f,0.5f };
	Vector3d resultVector = RotateVectorWithQuaternion(vector, quaternion);

	cout << "\tVector: " << vector.transpose() << endl;
	cout << "\tQuaternion: " << quaternion << endl << endl;

	cout << "\tRotated Vector: " << resultVector.transpose() << endl << endl;

	printf("EJ 3\n");
	srand(static_cast<unsigned int> (time(0)));

	int num_steps = 100;
	double start_angle = 0.0;
	double end_angle = 6.0 * PI;
	double step = (end_angle - start_angle) / (num_steps - 1);

	std::vector<double> angles;
	std::vector<double> traces;

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
		cout << "\tResults written to trace_vs_angle.csv" << endl << endl;
	}
	else {
		cerr << "\tUnable to open file for writing." << endl << endl;
		return 1;
	}

	printf("EJ 4\n");
	double roll = PI / 6;
	double pitch = PI / 3;
	double yaw = PI / 4;
	resultMatrix = RotationMatrix_FromEulerAngles(roll, pitch, yaw);
	cout << "\tRoll: " << roll <<endl;
	cout << "\tPitch: " << pitch <<endl;
	cout << "\tYaw: " << yaw <<endl << endl;
	cout << "\tResult RotationMatrix: " << endl << resultMatrix << endl << endl;
	resultVector = EulerAngles_FromRotationMatrix(resultMatrix);
	cout << "\tResult Vector From RotationMatrix: " << resultVector.transpose() << endl << endl;

	printf("EJ 5 & 6 -->");
	char inputType;
	cout << "\tInput Type {Use 'r', 'e', 'p', 'q', or 'v'}: ";
	cin >> inputType;
	cout << endl;


	Matrix3d R_in = Matrix3d::Identity();
	Vector3d euler_in = Vector3d::Zero();
	Vector3d axis_in = Vector3d::Zero();
	double angle_in = 0.0;
	Quaterniond quat_in = Quaterniond(0.707, 0, 0.707, 0);
	Vector3d rotVec_in = Vector3d::Zero();

	Convert(inputType, R_in, euler_in, axis_in, angle_in, quat_in, rotVec_in);

	cout << "\tResult RotationMatrix: " << endl << R_in << endl << endl;
	cout << "\tResult EulerAngles: " << euler_in.transpose() << endl << endl;
	cout << "\tResult Axis: " << axis_in.transpose() << endl;
	cout << "\tResult Angle: " << angle_in << endl << endl;
	cout << "\tResult Quaternion: " << quat_in<< endl << endl;
	cout << "\tResult RotationVector: " << rotVec_in.transpose() << endl << endl;*/


	return 0;
}


///// angle&axis -> quaternion 
///			q = (cos(an/2),sin(an/2)*ax1,sin(an/2)*ax2,sin(an/2)*ax3)
///
/// Rodriguez
///		R(u, φ) = I * cos(φ) + (1 − cos(φ)) * (u * uT) + [u]× * sin(φ)
/// 