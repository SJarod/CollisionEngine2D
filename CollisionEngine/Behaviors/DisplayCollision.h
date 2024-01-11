#ifndef _DISPLAY_COLLISION_H_
#define _DISPLAY_COLLISION_H_

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "RenderWindow.h"
#include "World.h"

#include <string>
#include <iostream>

class CDisplayCollision : public CBehavior
{
public:
	CPolygonPtr polyA;
	CPolygonPtr polyB;

private:
	virtual void Update(float frameTime) override
	{
		//gVars->pPhysicEngine->Activate(false);


		Vec2 col, normal, pen;
		float dist;
		if (polyA->CheckCollision(*polyB, col, normal, dist, pen))
		{
			gVars->pRenderer->DisplayTextWorld("Collision point", col);
			gVars->pRenderer->DisplayTextWorld("Penetration point", pen);

			gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(dist), 50, 50);

			gVars->pRenderer->DrawLine(col, col + normal * dist, 0.f, 0.f, 1.f);
		}
	}
};


#endif