#ifndef _MATHS_H_
#define _MATHS_H_

#define _USE_MATH_DEFINES
#include <math.h>


#define RAD2DEG(x) ((x)*(180.0f/(float)M_PI))
#define DEG2RAD(x) ((x)*((float)M_PI/180.0f))

template<typename T>
T Select(bool condition, T a, T b)
{
	return ((T)condition) * a + (1 - ((T)condition)) * b;
}

template<typename T>
T Min(T a, T b)
{
	return Select(a < b, a, b);
}

template<typename T>
T Max(T a, T b)
{
	return Select(a > b, a, b);
}

template<typename T>
T Clamp(T val, T min, T max)
{
	return Min(Max(val, min), max);
}

float Sign(float a);

float Random(float from, float to);
size_t Random(size_t from, size_t to);

struct Vec2
{
	union
	{
		struct { float x; float y; };
		struct { float min; float max; };
	};

	Vec2() : x(0.0f), y(0.0f){}

	Vec2(float _x, float _y) : x(_x), y(_y){}

	inline Vec2 operator+(const float& a) const
	{
		return Vec2(x + a, y + a);
	}

	inline Vec2 operator+(const Vec2& rhs) const
	{
		return Vec2(x + rhs.x, y + rhs.y);
	}

	inline Vec2& operator+=(const Vec2& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	inline Vec2 operator-() const
	{
		return Vec2(-x, -y);
	}

	inline Vec2 operator-(const Vec2& rhs) const
	{
		return Vec2(x - rhs.x, y - rhs.y);
	}

	inline Vec2& operator-=(const Vec2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	inline Vec2 operator*(float factor) const
	{
		return Vec2(x * factor, y * factor);
	}

	inline Vec2& operator*=(float factor)
	{
		*this = Vec2(x * factor, y * factor);
		return *this;
	}

	inline Vec2 operator/(float factor) const
	{
		return Vec2(x / factor, y / factor);
	}

	inline Vec2& operator/=(float factor)
	{
		*this = Vec2(x / factor, y / factor);
		return *this;
	}

	inline float operator|(const Vec2& rhs) const
	{
		return x * rhs.x + y * rhs.y;
	}

	inline float operator^(const Vec2& rhs) const
	{
		return x * rhs.y - y * rhs.x;
	}

	inline float GetLength() const
	{
		return sqrtf(x*x + y*y);
	}

	inline float GetRange() const
	{
		return max - min;
	}

	inline float GetCenter() const
	{
		return (x + y) / 2.f;
	}

	inline float GetSqrLength() const
	{
		return x*x + y*y;
	}

	inline void	Normalize()
	{
		float length = GetLength();
		x /= length;
		y /= length;
	}

	inline Vec2	Normalized() const
	{
		Vec2 res = *this;
		res.Normalize();
		return res;
	}

	inline void Reflect(Vec2 normal, float elasticity = 1.0f)
	{
		*this = *this - normal * (1.0f + elasticity) * (*this | normal);
	}

	inline Vec2 GetNormal() const
	{
		return Vec2(-y, x);
	}

	inline float Angle(const Vec2& to) const
	{
		float cosAngle = Clamp(Normalized() | to.Normalized(), -1.0f, 1.0f);
		float angle = RAD2DEG(acosf(cosAngle)) * Sign(*this ^ to);
		return angle;
	}

	inline bool CheckRangeCollision(const Vec2& v)
	{
		return v.min >= min && v.min <= max ||
			min >= v.min && min <= v.max;
	}

	inline Vec2 Rotate(const float a)
	{
		Vec2 out;
		out.x = x * cosf(a) - y * sinf(a);
		out.y = x * sinf(a) + y * cosf(a);
		return out;
	}

	inline float Dot(const Vec2& v) const
	{
		return (x * v.x) + (y * v.y);
	}

	inline float Cross(const Vec2& v) const
	{
		return GetLength() * v.GetLength() * sinf(Angle(v));
	}
};


struct Vec3
{
	union
	{
		struct { float x; float y; float z; };
		Vec2 xy;
	};

	Vec3() {}
	Vec3(Vec2 v, float a) : x(v.x), y(v.y), z(a) {}
	Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

	inline float GetLength() const
	{
		return sqrtf(x * x + y * y + z * z);
	}

	inline float GetSqrLength() const
	{
		return x * x + y * y + z * z;
	}

	inline float Dot(const Vec3& v) const
	{
		return (x * v.x) + (y * v.y) + (z * v.z);
	}

	inline Vec3 Cross(const Vec3& v) const
	{
		Vec3 vv;
		vv.x = y * v.z - z * v.y;
		vv.y = z * v.x - x * v.z;
		vv.z = x * v.y - y * v.x;
		return vv;
	}

	inline Vec3 operator+(const Vec3& v) const
	{
		return Vec3(x + v.x, y + v.y, z + v.z);
	}

	inline Vec3 operator*(const float& a) const
	{
		return Vec3(x * a, y * a, z * a);
	}
};


struct Mat2
{
	Vec2 X, Y;

	Mat2() : X(1.0f, 0.0f), Y(0.0f, 1.0f){}

	Mat2(float a, float b, float c, float d) : X(a, c), Y(b, d){}

	Mat2	GetInverse() const
	{
		return Mat2(X.x, X.y, Y.x, Y.y);
	}

	float	GetAngle() const
	{
		return Vec2(1.0f, 0.0f).Angle(X);
	}

	void	SetAngle(float angle)
	{
		float c = cosf(angle * ((float)M_PI / 180.0f));
		float s = sinf(angle * ((float)M_PI / 180.0f));

		X.x = c; X.y = s;
		Y.x = -s; Y.y = c;
	}

	void Rotate(float angle)
	{
		Mat2 matRot;
		matRot.SetAngle(angle);
		*this = *this * matRot;
	}

	Mat2 operator*(const Mat2& rhs) const
	{
		return Mat2(X.x * rhs.X.x + Y.x * rhs.X.y,
			X.x * rhs.Y.x + Y.x * rhs.Y.y,
			X.y * rhs.X.x + Y.y * rhs.X.y,
			X.y * rhs.Y.x + Y.y * rhs.Y.y);
	}

	Mat2 operator*=(const Mat2& rhs)
	{
		*this = *this * rhs;
		return *this;
	}

	Vec2 operator*(const Vec2& vec) const
	{
		return Vec2(X.x*vec.x + Y.x*vec.y, X.y*vec.x + Y.y*vec.y);
	}

	Mat2 operator*(const float& a) const
	{
		return Mat2(X.x * a,
			Y.x * a,
			X.y * a,
			Y.y * a);
	}

	Mat2 operator*=(const float& a)
	{
		*this = *this * a;
		return *this;
	}

	Mat2 operator+(const Mat2& m) const
	{
		return Mat2(X.x + m.X.x,
			Y.x + m.Y.x,
			X.y + m.X.y,
			Y.y + m.Y.y);
	}

	Mat2 operator+=(const Mat2& m)
	{
		*this = *this + m;
		return *this;
	}
};

struct Line
{
	Vec2 point, dir;

	Line() = default;
	Line(Vec2 _point, Vec2 _dir) : point(_point), dir(_dir){}

	Vec2	GetNormal() const
	{
		return dir.GetNormal();
	}

	// positive value means point above line, negative means point is under line
	float	GetPointDist(const Vec2& pt) const
	{
		return (pt - point) | GetNormal();
	}

	Line	Transform(const Mat2& rotation, const Vec2& position) const
	{
		return Line(position + rotation * point, rotation * dir);
	}

	Vec2	Project(const Vec2& pt) const
	{
		return point + dir * ((pt - point) | dir);
	}
};





#endif