#include <deque>
#include <cassert>

#include "GlobalVariables.h"


#include "Polygon.h"
#include <GL/glu.h>


#include "PhysicEngine.h"


#define GJK_MAX_ITERATION 5
#define GJK_K 3


struct CollisionInfoT
{
	Vec2 a;
	Vec2 b;
	Vec2 normal;
	float distance;
};


void DrawLine(const Vec2& from, const Vec2& to, float r, float g, float b)
{
	glColor3f(r, g, b);
	glBegin(GL_LINES);
	glVertex3f(from.x, from.y, -1.0f);
	glVertex3f(to.x, to.y, -1.0f);
	glEnd();
}

void DrawPoint(const Vec2& v, float r, float g, float b)
{
	glColor3f(r, g, b);
	glLineWidth(3.f);
	glBegin(GL_LINES);
	glVertex3f(v.x, 0.5f + v.y, -1.f);
	glVertex3f(v.x, -0.5f + v.y, -1.f);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(0.5f + v.x, v.y, -1.f);
	glVertex3f(-0.5f + v.x, v.y, -1.f);
	glEnd();
	glLineWidth(1.f);
}



// narrow phase collision detection
bool Analytical(const CPolygon& a, const CPolygon& b, Vec2& colPoint, Vec2& colNormal, float& colDist)
{
	return false;
}

bool AxisRangeCheck(const Line& axis, const CPolygon& first, const CPolygon& second)
{
	Vec2 per = (first.rotation * axis.dir).GetNormal();
	Vec2 arange = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::lowest)() };
	Vec2 brange = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::lowest)() };
	for (auto& p : first.points)
	{
		float dot = per.Dot(first.position + first.rotation * p);
		arange.min = std::min(arange.min, dot);
		arange.max = std::max(arange.max, dot);
	}
	for (auto& p : second.points)
	{
		float dot = per.Dot(second.position + second.rotation * p);
		brange.min = std::min(brange.min, dot);
		brange.max = std::max(brange.max, dot);
	}

	return arange.CheckRangeCollision(brange);
}
bool SeparateAxisTheorem(const CPolygon& a, const CPolygon& b, Vec2& colPoint, Vec2& colNormal, float& colDist)
{
	for (auto& l : a.m_lines)
	{
		if (!AxisRangeCheck(l, a, b))
			return false;
	}
	for (auto& l : b.m_lines)
	{
		if (!AxisRangeCheck(l, b, a))
			return false;
	}

	return true;
}

bool PointLineIntersection(Vec2 p1, Vec2 p2, Vec2 v)
{
	Vec2 a = p2 - p1;
	Vec2 b = v - p1;
	if (a.GetSqrLength() == 0.f || b.GetSqrLength() == 0.f)
		return false;

	return (a.Dot(b) / (a.GetLength() * b.GetLength())) == 1.f;
}
// simplex origin
Vec2 SimplexOrigin(const std::deque<Vec2>& simplex)
{
	Vec2 sum = Vec2(0.f, 0.f);
	for (const Vec2& v : simplex)
	{
		sum += v;
	}
	return sum / simplex.size();
}
// simplex normal (undefined direction)
Vec2 SimplexDirection(const std::deque<Vec2>& simplex)
{
	assert(simplex.size() < GJK_K);

	// AB = B - A
	Vec2 d = *simplex.rbegin();
	Vec2 a = d;
	for (auto it = simplex.rbegin() + 1; it != simplex.rend(); ++it)
	{
		d -= *it;
	}
	Vec2 dir = Vec2{ -d.y, d.x }.Normalized();

	// check if found direction is pointing towards the origin
	float sign = (-a).Dot(dir);
	sign = sign < 0.f ? -1.f : 1.f;
	return dir * sign;
}
// TODO : simplex intersection function with global referential
// simplex intersection (with origin by default)
bool SimplexIntersection(const std::deque<Vec2>& simplex, const Vec2& p = { 0.f, 0.f })
{
	float sign = 0.f;
	for (int i = 0; i < simplex.size(); ++i)
	{
		Vec2 a = simplex[i];
		Vec2 b = simplex[(i + 1) % simplex.size()];
		Vec2 ab = b - a;
		Vec2 n = Vec2{ -ab.y, ab.x }.Normalized();

		Vec2 ap = (p - a).Normalized();
		float s = n.Dot(ap);
		s /= std::abs(s);
		if (sign == 0.f)
			sign = s;
		else if (s != sign)
			return false;
	}
	return true;
}
bool GilbertJohnsonKeerthi(const CPolygon& polyA, const CPolygon& polyB, std::deque<Vec2>& simplex)
{
	if (gVars->bDebug)
		DrawPoint({ 0.f, 0.f }, 0.f, 0.f, 0.f);

	Vec2 dir0 = Vec2(1.f, 0.f);
	Vec2 supportA0 = polyA.SupportFunction(dir0);
	Vec2 supportB0 = polyB.SupportFunction(-dir0);
	// minkowski difference
	Vec2 diff0 = supportA0 - supportB0;

	if (gVars->bDebug)
	{
		DrawPoint(supportA0, 0.f, 1.f, 0.f);
		DrawPoint(supportB0, 0.f, 0.f, 1.f);
	}

	simplex.push_back(diff0);

	Vec2 dir = -diff0.Normalized();
	for (int i = 0; i < GJK_MAX_ITERATION; ++i)
	{
		Vec2 supportA = polyA.SupportFunction(dir);
		Vec2 supportB = polyB.SupportFunction(-dir);
		// new minkowski difference
		Vec2 diff = supportA - supportB;


		// does line pass origin ?

#if 0
		// TODO : take intersection point between dir and simplex (line line intersection)
		Vec2 p = *(simplex.end() - 1);
		Vec2 va = diff - p;
		Vec2 vb = -p;
		float bLen = vb.GetLength();
		float aLenCosTheta = va.Dot(vb) / bLen;
		if (aLenCosTheta < bLen)
#else
		if (dir.Dot(-diff) >= 0.f)
			return false;
		else
			simplex.push_back(diff);
#endif

		if (gVars->bDebug)
		{
			if (simplex.size() == 3)
			{
				DrawLine(simplex[0], simplex[1], 1.f, 0.f, 0.f);
				DrawLine(simplex[1], simplex[2], 1.f, 0.f, 0.f);
				DrawLine(simplex[2], simplex[0], 1.f, 0.f, 0.f);
			}
		}
		if (gVars->bDebug)
		{
			int ii = 1;
			for (const Vec2& s : simplex)
			{
				DrawPoint(s, ii++ / (float)simplex.size(), 0.f, 0.f);
			}
		}

		if (SimplexIntersection(simplex))
			return true;

		// find oldest point with segment furthest distance from origin
		// and remove from simplex
		if (simplex.size() == GJK_K)
		{
			// TODO : make loop instead of "+1" and "+2"
			const Vec2& a = *simplex.begin();
			const Vec2& b = *(simplex.begin() + 1);
			const Vec2& c = *(simplex.begin() + 2);

			Vec2 ca = a - c;
			Vec2 cb = b - c;
			Vec2 co = -c;

			float dota = co.Dot(ca);
			float thetaa = std::acosf(dota / (co.GetLength() * ca.GetLength()));
			float da = co.GetLength() * std::sinf(thetaa);

			float dotb = co.Dot(cb);
			float thetab = std::acosf(dotb / (co.GetLength() * cb.GetLength()));
			float db = co.GetLength() * std::sinf(thetab);

			if (da < db)
				simplex.erase(simplex.begin() + 1);
			else
				simplex.erase(simplex.begin());
		}

		dir = SimplexDirection(simplex);
	}

	return false;
}

CollisionInfoT ExpandingPolytopeAlgorithm2D(const CPolygon& a, const CPolygon& b, const std::deque<Vec2>& simplex)
{
	assert(simplex.size() == GJK_K);

	auto polytope = simplex;

	constexpr const float fmax = std::numeric_limits<float>::max();
	constexpr const float fepsilon = 0.001f;


	float minDistance = fmax;
	Vec2 minNormal = Vec2{ 0.f, 0.f };
	int minIndex = 0;

	Vec2 A, B;

	while (minDistance == fmax)
	{
		for (int i = 0; i < polytope.size(); ++i)
		{
			int j = (i + 1) % polytope.size();

			Vec2 a = polytope[i];
			Vec2 b = polytope[j];
			Vec2 ab = b - a;
			Vec2 n = Vec2{ -ab.y, ab.x }.Normalized();

			float d = n.Dot(a);
			if (d < 0.f)
			{
				d = -d;
				n = -n;
			}

			if (d < minDistance)
			{
				minDistance = d;
				minNormal = n;
				minIndex = j;
			}
		}

		Vec2 supportA = a.SupportFunction(minNormal);
		Vec2 supportB = b.SupportFunction(-minNormal);
		Vec2 support = supportA - supportB;
		float sDistance = minNormal.Dot(support);

		A = supportA;
		B = supportB;

		if (std::fabsf(sDistance - minDistance) > fepsilon)
		{
			minDistance = fmax;
			polytope.insert(polytope.begin() + minIndex, support);
		}
	}

	return { A, B, minNormal, minDistance };
}


CPolygon::CPolygon(size_t index)
	: m_polygonVertexBufferId(0), m_index(index), density(0.1f)
{
	aabb = std::make_shared<CAxisAlignedBoundingBox>(*this);
}

CPolygon::~CPolygon()
{
	DestroyBuffers();
}

void CPolygon::Build()
{
	m_lines.clear();

	CreateBuffers();
	BuildLines();
}

void CPolygon::Draw() const
{
	// polygon

	// Set transforms (assuming model view mode is set)
	float polygonTRS[16] = { rotation.X.x, rotation.X.y, 0.f, 0.f,
							 rotation.Y.x, rotation.Y.y, 0.f, 0.f,
							 0.f, 0.f, 0.f, 1.f,
							 position.x, position.y, -1.f, 1.f };
	glPushMatrix();
	glMultMatrixf(polygonTRS);

	if (!bCollisionWithOtherPolygon)
		glColor3f(0.5f, 0.f, 0.f);
	else
		glColor3f(0.f, 0.5f, 0.f);

	// Draw vertices
	BindPolygonBuffers();
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

	glColor3f(0.f, 0.f, 0.f);

	glPopMatrix();


	// aabb

	if (!gVars->bDebug)
		return;

	float aabbTRS[16] = { aabb->xrange.GetRange() / 2.f, 0.f, 0.f, 0.f,
						  0.f, aabb->yrange.GetRange() / 2.f, 0.f, 0.f,
						  0.f, 0.f, 1.f, 1.f,
						  aabb->xrange.GetCenter(), aabb->yrange.GetCenter(), -1.f, 1.f };
	glPushMatrix();
	glMultMatrixf(aabbTRS);

	if (!aabb->bCollisionWithOtherAABB)
		glColor3f(1.f, 0.f, 0.f);
	else
		glColor3f(0.f, 1.f, 0.f);

	// Draw vertices
	BindAABBBuffers();
	glDrawArrays(GL_LINE_LOOP, 0, 4);
	glDisableClientState(GL_VERTEX_ARRAY);

	glColor3f(0.f, 0.f, 0.f);

	glPopMatrix();
}

size_t CPolygon::GetIndex() const
{
	return m_index;
}

Vec2 CPolygon::TransformPoint(const Vec2& point) const
{
	return position + rotation * point;
}

Vec2 CPolygon::InverseTransformPoint(const Vec2& point) const
{
	return rotation.GetInverse() * (point - position);
}

bool CPolygon::IsPointInside(const Vec2& point) const
{
	float maxDist = -FLT_MAX;

	for (const Line& line : m_lines)
	{
		Line globalLine = line.Transform(rotation, position);
		float pointDist = globalLine.GetPointDist(point);
		maxDist = Max(maxDist, pointDist);
	}

	return maxDist <= 0.001f;
}

Vec2 CPolygon::SupportFunction(const Vec2& dir) const
{
	Vec2 support;
	float maxProjection = std::numeric_limits<float>::min();

	for (const auto& p : points)
	{
		Vec2 pp = rotation * p;
		float projection = pp.Dot(dir);
		if (projection > maxProjection)
		{
			maxProjection = projection;
			support = pp;
		}
	}

	return position + support;
}

bool CPolygon::CheckCollision(CPolygon& poly,
	Vec2& colPoint,
	Vec2& colNormal,
	float& colDist,
	Vec2& penPoint)
{
	// collision detection
	std::deque<Vec2> simplex;
	if (GilbertJohnsonKeerthi(*this, poly, simplex))
	{
		bCollisionWithOtherPolygon = true;
		poly.bCollisionWithOtherPolygon = true;

		// GJK extension
		auto info = ExpandingPolytopeAlgorithm2D(*this, poly, simplex);
		if (gVars->bDebug)
		{
			DrawPoint(info.a, 1.f, 0.f, 1.f);
			DrawPoint(info.b, 1.f, 1.f, 0.f);
		}

		// collision point reconstruction
		colPoint = info.a - info.normal * info.distance;
		penPoint = info.a;
		colNormal = info.normal;
		// if point is not inside checked polygon, use other polygon collision point
		// and invert normal
		if (!poly.IsPointInside(colPoint))
		{
			colNormal = -info.normal;
			colPoint = info.b - colNormal * info.distance;
			penPoint = info.b;
		}
		colDist = info.distance;
	}
	return bCollisionWithOtherPolygon;
}

void CPolygon::CreateBuffers()
{
	DestroyBuffers();


	// polygon
	{
		float* vertices = new float[3 * points.size()];
		for (size_t i = 0; i < points.size(); ++i)
		{
			vertices[3 * i] = points[i].x;
			vertices[3 * i + 1] = points[i].y;
			vertices[3 * i + 2] = 0.f;
		}

		glGenBuffers(1, &m_polygonVertexBufferId);

		glBindBuffer(GL_ARRAY_BUFFER, m_polygonVertexBufferId);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * points.size(), vertices, GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		delete[] vertices;
	}


	// aabb
	{
		float* vertices = new float[3 * 4];
		vertices[0] = -1.f;
		vertices[1] = -1.f;
		vertices[2] = 0.f;
		vertices[3] = 1.f;
		vertices[4] = -1.f;
		vertices[5] = 0.f;
		vertices[6] = 1.f;
		vertices[7] = 1.f;
		vertices[8] = 0.f;
		vertices[9] = -1.f;
		vertices[10] = 1.f;
		vertices[11] = 0.f;

		glGenBuffers(1, &m_aabbVertexBufferId);

		glBindBuffer(GL_ARRAY_BUFFER, m_aabbVertexBufferId);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * 4, vertices, GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		delete[] vertices;
	}
}

void CPolygon::BindPolygonBuffers() const
{
	if (m_polygonVertexBufferId != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_polygonVertexBufferId);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, (void*)0);
	}
}

void CPolygon::BindAABBBuffers() const
{
	if (m_aabbVertexBufferId != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_aabbVertexBufferId);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, (void*)0);
	}
}


void CPolygon::DestroyBuffers()
{
	if (m_polygonVertexBufferId != 0)
	{
		glDeleteBuffers(1, &m_polygonVertexBufferId);
		m_polygonVertexBufferId = 0;
	}
	if (m_aabbVertexBufferId != 0)
	{
		glDeleteBuffers(1, &m_aabbVertexBufferId);
		m_aabbVertexBufferId = 0;
	}
}

void CPolygon::BuildLines()
{
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];

		Vec2 lineDir = (pointA - pointB).Normalized();

		m_lines.push_back(Line(pointB, lineDir));
	}
}
