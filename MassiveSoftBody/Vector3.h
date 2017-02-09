
#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>

template <typename Real>
struct Vector3 {
	Vector3(Real x = Real(0), Real y = Real(0), Real z = Real(0)) {
		this->data[0] = x;
		this->data[1] = y;
		this->data[2] = z;
	}

	Vector3 operator - () {
		return Vector3(-this->data[0], -this->data[1], -this->data[2]);
	}

	Vector3 operator - (const Vector3<Real>& v) {
		return Vector3(this->data[0] - v.data[0], this->data[1] - v.data[1], this->data[2] - v.data[2]);
	}

	Vector3 operator + (const Vector3<Real>& v) {
		return Vector3(this->data[0] + v.data[0], this->data[1] + v.data[1], this->data[2] + v.data[2]);
	}

	operator const float* const () const { return this->data; }

	static Vector3 Cross(const Vector3<Real>& a, const Vector3<Real>& b) {
		Vector3 result;
		result.data[0] = ((a.data[1] * b.data[2]) - (a.data[2] * b.data[1]));
		result.data[1] = -((a.data[0] * b.data[2]) - (a.data[2] * b.data[0]));
		result.data[2] = ((a.data[0] * b.data[1]) - (a.data[1] * b.data[0]));
		return result;
	}

	static float Length(const Vector3<Real>& vector) {
		return std::sqrt(vector.data[0] * vector.data[0] + vector.data[1] * vector.data[1] + vector.data[2] * vector.data[2]); 
	}

	static void Normalize(Vector3<Real>& vector) {
		float invLen = 1.0f / Length(vector);
		vector.data[0] *= invLen;
		vector.data[1] *= invLen;
		vector.data[2] *= invLen;
	}

	Real data[3];
};

typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

#endif
