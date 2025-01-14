#include "PhysicEngine.h"

#include <iostream>
#include <string>
#include "GlobalVariables.h"
#include "World.h"
#include "Renderer.h" // for debugging only
#include "Timer.h"

#include "BroadPhase.h"
#include "BroadPhaseBrut.h"
#include "BroadPhaseAABBIntersection.h"
#include "BroadPhaseSweepAndPrune.h"


void	CPhysicEngine::Reset()
{
	m_pairsToCheck.clear();
	m_collidingPairs.clear();

	m_active = true;

	//m_broadPhase = new CBroadPhaseBrut();
	//m_broadPhase = new CBroadPhaseAABBIntersection();
	m_broadPhase = new CBroadPhaseSweepAndPrune();
}

void	CPhysicEngine::Activate(bool active)
{
	m_active = active;
}

bool CPhysicEngine::IsActive() const
{
	return m_active;
}

void	CPhysicEngine::DetectCollisions()
{
	CTimer timer;
	timer.Start();
	CollisionBroadPhase();
	timer.Stop();
	if (gVars->bDebug)
	{
		gVars->pRenderer->DisplayText("Collision broadphase duration " + std::to_string(timer.GetDuration() * 1000.0f) + " ms");
	}

	timer.Start();
	CollisionNarrowPhase();
	timer.Stop();
	if (gVars->bDebug)
	{
		gVars->pRenderer->DisplayText("Collision narrowphase duration " + std::to_string(timer.GetDuration() * 1000.0f) + " ms, collisions : " + std::to_string(m_collidingPairs.size()));
	}
}

void	CPhysicEngine::Step(float deltaTime)
{
	if (!m_active)
	{
		return;
	}

	DetectCollisions();
}

void	CPhysicEngine::CollisionBroadPhase()
{
	gVars->pWorld->ForEachPolygon([](CPolygonPtr poly) { poly->aabb->Reset(*poly); });
	m_pairsToCheck.clear();
	m_broadPhase->GetCollidingPairsToCheck(m_pairsToCheck);
}

void	CPhysicEngine::CollisionNarrowPhase()
{
	gVars->pWorld->ForEachPolygon([](CPolygonPtr poly) { poly->bCollisionWithOtherPolygon = false; });
	m_collidingPairs.clear();
	for (const SPolygonPair& pair : m_pairsToCheck)
	{
		SCollision collision;
		collision.polyA = pair.polyA;
		collision.polyB = pair.polyB;
		if (collision.polyA->CheckCollision(*(collision.polyB),
			collision.collision,
			collision.normal,
			collision.distance,
			collision.penetration))
		{
			m_collidingPairs.push_back(collision);
		}
	}
}