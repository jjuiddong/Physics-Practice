#pragma once

#include "../../../Common/Common/common.h"
using namespace common;
#include "../../../Common/Graphic11/graphic11.h"
#include "../../../Common/Framework11/framework11.h"
#include "PxPhysicsAPI.h"

// physx release macro
#if !defined(PX_SAFE_RELEASE)
	#define PX_SAFE_RELEASE(p) {if((p)) {p->release(); p = nullptr;}}
#endif


#include "physics.h"
extern cPhysicsEngine g_physics;
