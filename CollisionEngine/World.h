#ifndef _WORLD_H_
#define _WORLD_H_

#include <vector>

#include "Polygon.h"
#include "Behavior.h"

struct SRandomPolyParams
{
	size_t	minPoints = 0, maxPoints = 0;
	float	minRadius = 0.f, maxRadius = 0.f;
	Vec2	minBounds, maxBounds;
	float	minSpeed = 0.f, maxSpeed = 0.f;
};

class CWorld
{
public:
	CPolygonPtr		AddTriangle(float base, float height);
	CPolygonPtr		AddRectangle(float width, float height);
	CPolygonPtr		AddSquare(float size);
	CPolygonPtr		AddSymetricPolygon(float radius, size_t sides);
	CPolygonPtr		AddRandomPoly(const SRandomPolyParams& params);

	CPolygonPtr		AddPolygon();
	void			RemovePolygon(CPolygonPtr poly);

	template<class TBehavior>
	CBehaviorPtr	AddBehavior(CPolygonPtr poly)
	{
		CBehaviorPtr behavior(new TBehavior());
		behavior->m_index = m_behaviors.size();
		behavior->poly = poly;
		m_behaviors.push_back(behavior);

		return behavior;
	}
	void			RemoveBehavior(CBehaviorPtr behavior);

	template<typename TFunctor>
	void	ForEachPolygon(TFunctor functor)
	{
		for (CPolygonPtr poly : m_polygons)
		{
			functor(poly);
		}
	}
	size_t		GetPolygonCount() const;
	CPolygonPtr&	GetPolygon(size_t index);
	inline const std::vector<CPolygonPtr>& GetPolygonArray() const { return m_polygons; }

	template<typename TFunctor>
	void	ForEachBehavior(TFunctor functor)
	{
		for (CBehaviorPtr behavior : m_behaviors)
		{
			functor(behavior);
		}
	}

	void Update(float frameTime);
	void RenderPolygons();

protected:
	std::vector<CPolygonPtr>	m_polygons;
	std::vector<CBehaviorPtr>	m_behaviors;
};

#endif