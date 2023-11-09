#ifndef _PHYSIC_ENGINE_H_
#define _PHYSIC_ENGINE_H_

#include <vector>
#include <unordered_map>
#include "Maths.h"
#include "Polygon.h"

class IBroadPhase;

struct SPolygonPair
{
	CPolygonPtr	polyA;
	CPolygonPtr	polyB;

	SPolygonPair(CPolygonPtr _polyA, CPolygonPtr _polyB) : polyA(_polyA), polyB(_polyB) {}
	inline const bool AABBIntersection() const
	{
		CAxisAlignedBoundingBox& a = *polyA->aabb;
		CAxisAlignedBoundingBox& b = *polyB->aabb;

		// x axis
		bool bXAxisIntersect = b.xrange.min >= a.xrange.min && b.xrange.min <= a.xrange.max ||
			a.xrange.min >= b.xrange.min && a.xrange.min <= b.xrange.max;
		if (!bXAxisIntersect)
			return false;

		bool bYAxisIntersect = b.yrange.min >= a.yrange.min && b.yrange.min <= a.yrange.max ||
			a.yrange.min >= b.yrange.min && a.yrange.min <= b.yrange.max;

		return bYAxisIntersect;
	}
};

struct SCollision
{
	CPolygonPtr	polyA, polyB;

	Vec2	point;
	Vec2	normal;
	float	distance;

	SCollision() = default;
	SCollision(CPolygonPtr _polyA, CPolygonPtr _polyB, Vec2	_point, Vec2 _normal, float _distance)
		: polyA(_polyA), polyB(_polyB), point(_point), normal(_normal), distance(_distance) {}
};

class CPhysicEngine
{
public:
	void	Reset();
	void	Activate(bool active);

	void	DetectCollisions();

	void	Step(float deltaTime);

	template<typename TFunctor>
	void	ForEachCollision(TFunctor functor)
	{
		for (const SCollision& collision : m_collidingPairs)
		{
			functor(collision);
		}
	}

private:
	friend class CPenetrationVelocitySolver;

	void						CollisionBroadPhase();
	void						CollisionNarrowPhase();

	bool						m_active = true;

	// Collision detection
	IBroadPhase* m_broadPhase = NULL;
	std::vector<SPolygonPair>	m_pairsToCheck;
	std::vector<SCollision>		m_collidingPairs;
};

#endif