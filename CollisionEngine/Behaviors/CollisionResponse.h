#ifndef _COLLISION_RESPONSE_H_
#define _COLLISION_RESPONSE_H_

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"

class CCollisionResponse : public CBehavior
{
private:
	virtual void Update(float frameTime) override
	{
		gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
			{
				Vec2 normal = -collision.normal;

				float ainvMass = 1.f / collision.polyA->mass;
				float binvMass = 1.f / collision.polyB->mass;

				// position correction

				float damping = 0.2f;
				float correction = (collision.distance * damping) / (ainvMass + binvMass);

				collision.polyA->position += normal * ainvMass * correction;
				collision.polyB->position -= normal * binvMass * correction;

				// velocity impulse

				float restitution = 1.f;

				Vec2 rA = collision.collision - collision.polyA->position;
				Vec2 rB = collision.collision - collision.polyB->position;
				Vec2 aspeed = collision.polyA->speed;
				Vec2 bspeed = collision.polyB->speed;
				Vec2 vAi = aspeed + rA * collision.polyA->angularSpeed;
				Vec2 vBi = bspeed + rB * collision.polyB->angularSpeed;
				//Vec2 momentumA = 

				// are polygons going towards each other ?
				float vRel = (aspeed - bspeed).Dot(normal);
				if (vRel >= 0.f)
					return;

				// velocity

				float J = (-(1 + restitution) * vRel) / (ainvMass + binvMass);

				collision.polyA->speed += normal * J * ainvMass;
				collision.polyB->speed -= normal * J * binvMass;

				// angular velocity

				float alocalTensor = collision.polyA->mass * collision.collision.GetSqrLength();
				float ainvlocalTensor = 1.f / alocalTensor;

				Mat2 ainvWorldTensor = collision.polyA->rotation * ainvlocalTensor *
					collision.polyA->rotation.GetInverse();
			});

		float hWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
		float hHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

		gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
			{
				poly->position += poly->speed * frameTime;
				poly->rotation += poly->rotation * poly->angularSpeed * frameTime;

				if (poly->position.x < -hWidth)
				{
					poly->position.x = -hWidth;
					poly->speed.x *= -1.0f;
				}
				else if (poly->position.x > hWidth)
				{
					poly->position.x = hWidth;
					poly->speed.x *= -1.0f;
				}
				if (poly->position.y < -hHeight)
				{
					poly->position.y = -hHeight;
					poly->speed.y *= -1.0f;
				}
				else if (poly->position.y > hHeight)
				{
					poly->position.y = hHeight;
					poly->speed.y *= -1.0f;
				}
			});
	}
};

#endif