#ifndef _BROAD_PHASE_SWEEP_AND_PRUNE_H_
#define _BROAD_PHASE_SWEEP_AND_PRUNE_H_

#include "BroadPhase.h"

#include "Polygon.h"
#include "GlobalVariables.h"
#include "World.h"

class CBroadPhaseSweepAndPrune : public IBroadPhase
{
public:
	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override
	{
		for (size_t i = 0; i < gVars->pWorld->GetPolygonCount(); ++i)
		{
			for (size_t j = i + 1; j < gVars->pWorld->GetPolygonCount(); ++j)
			{
				auto& a = gVars->pWorld->GetPolygon(i);
				auto& b = gVars->pWorld->GetPolygon(j);
				auto p = SPolygonPair(a, b);
				if (p.AABBIntersection())
				{
					a->aabb->bCollisionWithOtherAABB = true;
					b->aabb->bCollisionWithOtherAABB = true;
					pairsToCheck.push_back(p);
				}
			}
		}
	}
};

#endif