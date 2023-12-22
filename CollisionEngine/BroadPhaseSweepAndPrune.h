#ifndef _BROAD_PHASE_SWEEP_AND_PRUNE_H_
#define _BROAD_PHASE_SWEEP_AND_PRUNE_H_

#include <algorithm>


#include "BroadPhase.h"

#include "Polygon.h"
#include "GlobalVariables.h"
#include "World.h"

class CBroadPhaseSweepAndPrune : public IBroadPhase
{
public:
	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override
	{
		// sort with x min
		std::vector<CPolygonPtr> sorted = gVars->pWorld->GetPolygonArray();
		std::sort(sorted.begin(), sorted.end(), [](CPolygonPtr a, CPolygonPtr b) -> bool {
			return a->aabb->xrange.min < b->aabb->xrange.min;
			});

		for (size_t i = 0; i < sorted.size(); ++i)
		{
			for (size_t j = i + 1; j < sorted.size(); ++j)
			{
				CPolygonPtr a = sorted[i];
				CPolygonPtr b = sorted[j];

				float amaxx = a->aabb->xrange.max;
				float bminx = b->aabb->xrange.min;

				// AABB overlap on x axis
				if (bminx <= amaxx)
				{
					// check for overlap on y axis

					// sort
					if (a->aabb->yrange.min > b->aabb->yrange.min)
					{
						a = sorted[j];
						b = sorted[i];
					}


					float amaxy = a->aabb->yrange.max;
					float bminy = b->aabb->yrange.min;

					// AABB overlap on y axis
					if (bminy <= amaxy)
					{
						a->aabb->bCollisionWithOtherAABB = true;
						b->aabb->bCollisionWithOtherAABB = true;

						auto p = SPolygonPair(a, b);
						pairsToCheck.push_back(p);
					}
				}
				else
				{
					break;
				}
			}
		}
	}
};

#endif