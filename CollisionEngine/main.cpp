// main.cpp�: d�finit le point d'entr�e pour l'application console.
//

#include <iostream>
#include <string>

#include "Application.h"
#include "SceneManager.h"
#include "Scenes/SceneDebugCollisions.h"
#include "Scenes/SceneBouncingPolys.h"
#include "Scenes/SceneSpheres.h"


/*
* Entry point
*/
int main(int argc, char** argv)
{
	InitApplication(1366, 768, 50.0f);

	gVars->pSceneManager->AddScene(new CSceneDebugCollisions());
	gVars->pSceneManager->AddScene(new CSceneBouncingPolys(999));
	gVars->pSceneManager->AddScene(new CSceneSpheres());


	RunApplication();
	return 0;
}

