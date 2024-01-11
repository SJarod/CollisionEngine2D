#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <GL/glew.h>
#include <vector>
#include <memory>

#include <limits>


#include "Maths.h"



class CPolygon
{
private:
	friend class CWorld;

	CPolygon(size_t index);
public:
	~CPolygon();

	Vec2				position;
	Mat2				rotation;
	std::vector<Vec2>	points;


	void				Build();
	void				Draw() const;
	size_t				GetIndex() const;

	float				GetArea() const;

	Vec2				TransformPoint(const Vec2& point) const;
	Vec2				InverseTransformPoint(const Vec2& point) const;

	// if point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const Vec2& point) const;

	Vec2				SupportFunction(const Vec2& dir) const;
	/**
	 * Collision detection algorithm
	 * 
	 * @param polygon to check against
	 * @param collision point
	 * @param collision normal
	 * @param collision distance (normal length)
	 * @param penetration point
	 */
	bool				CheckCollision(CPolygon& poly,
									   Vec2& colPoint,
									   Vec2& colNormal,
									   float& colDist,
									   Vec2& penPoint);

	// Physics
	float				density;
	Vec2				speed;

private:
	void				CreateBuffers();
	void				BindPolygonBuffers() const;
	void				BindAABBBuffers() const;
	void				DestroyBuffers();

	void				BuildLines();

	GLuint				m_polygonVertexBufferId;
	GLuint				m_aabbVertexBufferId;
	size_t				m_index;

public:
	std::vector<Line>	m_lines;

	std::shared_ptr<class CAxisAlignedBoundingBox> aabb;
	bool bCollisionWithOtherPolygon = false;
};


typedef std::shared_ptr<CPolygon>	CPolygonPtr;




class CAxisAlignedBoundingBox
{
public:
	bool bCollisionWithOtherAABB = false;

	Vec2 xrange = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::lowest)() };
	Vec2 yrange = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::lowest)() };

public:
	CAxisAlignedBoundingBox() = delete;
	CAxisAlignedBoundingBox(const class CPolygon& polygon)
	{
		Reset(polygon);
	}

	void Reset(const class CPolygon& polygon)
	{
		bCollisionWithOtherAABB = false;
		xrange = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::lowest)() };
		yrange = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::lowest)() };

		for (auto& p : polygon.points)
		{
			Vec2 v = polygon.rotation * p;
			xrange.x = Min(xrange.x, polygon.position.x + v.x);
			xrange.y = Max(xrange.y, polygon.position.x + v.x);
			yrange.x = Min(yrange.x, polygon.position.y + v.y);
			yrange.y = Max(yrange.y, polygon.position.y + v.y);
		}
	}
};



#endif