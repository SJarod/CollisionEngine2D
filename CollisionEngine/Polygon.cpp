#include "GlobalVariables.h"


#include "Polygon.h"
#include <GL/glu.h>


#include "PhysicEngine.h"

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

	// Draw vertices
	BindPolygonBuffers();
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

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

size_t	CPolygon::GetIndex() const
{
	return m_index;
}

Vec2	CPolygon::TransformPoint(const Vec2& point) const
{
	return position + rotation * point;
}

Vec2	CPolygon::InverseTransformPoint(const Vec2& point) const
{
	return rotation.GetInverse() * (point - position);
}

bool	CPolygon::IsPointInside(const Vec2& point) const
{
	float maxDist = -FLT_MAX;

	for (const Line& line : m_lines)
	{
		Line globalLine = line.Transform(rotation, position);
		float pointDist = globalLine.GetPointDist(point);
		maxDist = Max(maxDist, pointDist);
	}

	return maxDist <= 0.f;
}

bool	CPolygon::CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const
{
	return false;
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
