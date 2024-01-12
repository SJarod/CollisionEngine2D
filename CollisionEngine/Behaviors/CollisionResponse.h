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

				Vec3 vAi3 = Vec3(aspeed, 0.f) + Vec3{ 0.f, 0.f, collision.polyA->angularSpeed }.Cross(Vec3(rA, 0.f));
				Vec3 vBi3 = Vec3(bspeed, 0.f) + Vec3{ 0.f, 0.f, collision.polyB->angularSpeed }.Cross(Vec3(rB, 0.f));
				Vec2 vAi = Vec2(vAi3.x, vAi3.y);
				Vec2 vBi = Vec2(vBi3.x, vBi3.y);

				float alocalTensor = collision.polyA->mass * collision.collision.GetSqrLength();
				float ainvlocalTensor = 1.f / alocalTensor;
				float blocalTensor = collision.polyB->mass * collision.collision.GetSqrLength();
				float binvlocalTensor = 1.f / blocalTensor;

				float ainvWorldTensor = collision.polyA->rotation.GetAngle() * ainvlocalTensor *
					collision.polyA->rotation.GetInverse().GetAngle();
				float binvWorldTensor = collision.polyB->rotation.GetAngle() * binvlocalTensor *
					collision.polyB->rotation.GetInverse().GetAngle();

				Vec3 momentumA = Vec3(rA, 0.f).Cross(Vec3(normal, 0.f)) * ainvWorldTensor;
				Vec3 momentumB = Vec3(rB, 0.f).Cross(Vec3(normal, 0.f)) * binvWorldTensor;
				float weightRotA = momentumA.Cross(Vec3(rA, 0.f)).Dot(Vec3(normal, 0.f));
				float weightRotB = momentumB.Cross(Vec3(rB, 0.f)).Dot(Vec3(normal, 0.f));

				// are polygons going towards each other ?
				//float vRel = (vAi - vBi).Dot(normal);
				float vRel = (aspeed - bspeed).Dot(normal);
				if (vRel >= 0.f)
					return;

				//float J = (-(1 + restitution) * vRel) / (ainvMass + binvMass + weightRotA + weightRotB);
				float J = (-(1 + restitution) * vRel) / (ainvMass + binvMass);

				// velocity

				collision.polyA->speed += normal * J * ainvMass;
				collision.polyB->speed -= normal * J * binvMass;

				// angular velocity

				collision.polyA->angularSpeed += J * momentumA.GetLength();
				collision.polyB->angularSpeed -= J * momentumB.GetLength();
			});

		float hWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
		float hHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

		gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
			{
				// translation
				poly->position += poly->speed * frameTime;
#if 0
				// rotation
				Mat2 m = Mat2();
#if 0
				m.SetAngle(poly->rotation.GetAngle() + poly->angularSpeed * frameTime);
				poly->rotation = m;
#else
				m.SetAngle(poly->angularSpeed * frameTime);
				poly->rotation *= m;
#endif
#endif

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