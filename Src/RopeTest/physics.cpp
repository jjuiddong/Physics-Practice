
#include "stdafx.h"
#include "physics.h"

using namespace physx;
using namespace graphic;


struct sPvdParameters
{
	string ip;
	int port;
	int timeout;
	bool useFullPvdConnection;
	sPvdParameters()
		: port(5425)
		, timeout(10)
		, useFullPvdConnection(true)
		, ip("127.0.0.1")
	{
	}
};


cPhysicsEngine::cPhysicsEngine()
{
}

cPhysicsEngine::~cPhysicsEngine()
{
	for (auto &p : m_actors)
	{
		m_scene->removeActor(*p.actor);
		PX_SAFE_RELEASE(p.actor);
		SAFE_DELETE(p.node);
	}
	m_actors.clear();

	for (auto &j : m_joints)
		PX_SAFE_RELEASE(j);
	m_joints.clear();

	m_physics->unregisterDeletionListener(*this);
	PX_SAFE_RELEASE(m_scene);
	PX_SAFE_RELEASE(m_cudaContextManager);
	PX_SAFE_RELEASE(m_cpuDispatcher);
	PX_SAFE_RELEASE(m_cooking);
	PX_SAFE_RELEASE(m_material);
	PX_SAFE_RELEASE(m_physics);
	PX_SAFE_RELEASE(m_pvd);
	PX_SAFE_RELEASE(m_transport);
	PX_SAFE_RELEASE(m_foundation);
	_aligned_free(m_bufferedActiveTransforms);
}


bool cPhysicsEngine::InitializePhysx(cRenderer &renderer)
{
	m_foundation = PxCreateFoundation(PX_FOUNDATION_VERSION
		, m_defaultAllocatorCallback, m_defaultErrorCallback);

	// pvd connection
	sPvdParameters pvdParams;
	m_transport = physx::PxDefaultPvdSocketTransportCreate(pvdParams.ip.c_str(), pvdParams.port
		, pvdParams.timeout);
	if (m_transport == NULL)
		return false;
	m_pvdFlags = physx::PxPvdInstrumentationFlag::eALL;
	if (!pvdParams.useFullPvdConnection)
		m_pvdFlags = physx::PxPvdInstrumentationFlag::ePROFILE;
	m_pvd = physx::PxCreatePvd(*m_foundation);
	m_pvd->connect(*m_transport, m_pvdFlags);
	//~pvd

	bool recordMemoryAllocations = true;
	physx::PxTolerancesScale scale;
	m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, scale
		, recordMemoryAllocations, m_pvd);

	m_physics->registerDeletionListener(*this, physx::PxDeletionEventFlag::eUSER_RELEASE);

	m_material = m_physics->createMaterial(0.5f, 0.5f, 0.1f);
	if (!m_material)
		return false;

	physx::PxCookingParams params(scale);
	params.meshWeldTolerance = 0.001f;
	params.meshPreprocessParams = physx::PxMeshPreprocessingFlags(physx::PxMeshPreprocessingFlag::eWELD_VERTICES);
	params.buildGPUData = true; //Enable GRB data being produced in cooking.
	m_cooking = PxCreateCooking(PX_PHYSICS_VERSION, *m_foundation, params);
	if (!m_cooking)
		return false;

	// scene initialize
	physx::PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
	if (!sceneDesc.cpuDispatcher)
	{
		const uint numThreads = 1;
		m_cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(numThreads);
		if (!m_cpuDispatcher)
			return false;
		sceneDesc.cpuDispatcher = m_cpuDispatcher;
	}

	if (!sceneDesc.filterShader)
		sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;

	physx::PxCudaContextManagerDesc cudaContextManagerDesc;
	m_cudaContextManager = PxCreateCudaContextManager(*m_foundation, cudaContextManagerDesc);
	if (m_cudaContextManager)
	{
		if (!m_cudaContextManager->contextIsValid())
		{
			PX_SAFE_RELEASE(m_cudaContextManager);
		}
	}
	if (!sceneDesc.gpuDispatcher && m_cudaContextManager)
		sceneDesc.gpuDispatcher = m_cudaContextManager->getGpuDispatcher();

	//sceneDesc.frictionType = physx::PxFrictionType::eTWO_DIRECTIONAL;
	//sceneDesc.frictionType = physx::PxFrictionType::eONE_DIRECTIONAL;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_PCM;
	//sceneDesc.flags |= physx::PxSceneFlag::eENABLE_AVERAGE_POINT;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_STABILIZATION;
	//sceneDesc.flags |= physx::PxSceneFlag::eADAPTIVE_FORCE;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_ACTIVETRANSFORMS;
	sceneDesc.sceneQueryUpdateMode = physx::PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_DISABLED;
	//sceneDesc.flags |= physx::PxSceneFlag::eDISABLE_CONTACT_CACHE;
	sceneDesc.broadPhaseType = physx::PxBroadPhaseType::eGPU;
	sceneDesc.gpuMaxNumPartitions = 8;

	m_scene = m_physics->createScene(sceneDesc);
	if (!m_scene)
		return false;

	physx::PxSceneWriteLock scopedLock(*m_scene);
	physx::PxSceneFlags flag = m_scene->getFlags();
	PX_UNUSED(flag);
	const bool IsinitialDebugRender = false;
	const float debugRenderScale = 1.f;
	m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eSCALE
		, IsinitialDebugRender ? debugRenderScale : 0.0f);
	m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

	physx::PxPvdSceneClient* pvdClient = m_scene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	return true;
}


bool cPhysicsEngine::PreUpdate(const float deltaSeconds)
{
	physx::PxSceneWriteLock writeLock(*m_scene);
	m_scene->simulate(0.01f, nullptr);

	return true;
}


bool cPhysicsEngine::PostUpdate(const float deltaSeconds)
{
	using namespace physx;

	// update physx result info
	{
		PxSceneWriteLock writeLock(*m_scene);
		m_scene->fetchResults(true);
	}

	// update active actor buffer
	uint activeActorSize = 0;
	{
		PxSceneReadLock scopedLock(*m_scene);
		const PxActiveTransform* activeTransforms =
			m_scene->getActiveTransforms(activeActorSize);
		if (activeActorSize > m_activeBufferCapacity)
		{
			_aligned_free(m_bufferedActiveTransforms);

			m_activeBufferCapacity = activeActorSize;
			m_bufferedActiveTransforms = (physx::PxActiveTransform*)_aligned_malloc(
				sizeof(physx::PxActiveTransform) * activeActorSize, 16);
		}

		if (activeActorSize > 0)
		{
			PxMemCopy(m_bufferedActiveTransforms, activeTransforms
				, sizeof(PxActiveTransform) * activeActorSize);
		}
	}

	// update render object
	for (uint i = 0; i < activeActorSize; ++i)
	{
		PxActiveTransform *activeTfm = &m_bufferedActiveTransforms[i];

		auto it = find_if(m_actors.begin(), m_actors.end(), [&](const auto &a) {
			return (a.actor == activeTfm->actor); });
		if (m_actors.end() == it)
			continue;

		sActor &actor = *it;
		{
			Transform tfm = actor.node->m_transform;
			tfm.pos = *(Vector3*)&activeTfm->actor2World.p;
			tfm.rot = *(Quaternion*)&activeTfm->actor2World.q;
			//Quaternion q = *(Quaternion*)&activeTfm->actor2World.q;
			//Matrix44 m = q.GetMatrix();

			actor.node->m_transform = tfm;
		}
	}

	return true;
}


physx::PxRigidActor* cPhysicsEngine::CreateGrid()
{
	PxSceneWriteLock scopedLock(*m_scene);

	PxRigidStatic* plane = PxCreatePlane(*m_physics
		, PxPlane(PxVec3(0, 1, 0), 0), *m_material);
	if (!plane)
		return nullptr;

	m_scene->addActor(*plane);

	//PxShape* shape;
	//plane->getShapes(&shape, 1);
	return plane;
}


physx::PxRigidDynamic* cPhysicsEngine::CreateBox(const Vector3& pos
	, const Vector3& dims
	, const Vector3* linVel // = nullptr
	, float density // = 1.f
)
{
	PxSceneWriteLock scopedLock(*m_scene);
	PxRigidDynamic* box = PxCreateDynamic(*m_physics, PxTransform(*(PxVec3*)&pos)
		, PxBoxGeometry(*(PxVec3*)&dims), *m_material, density);
	PX_ASSERT(box);

	box->setActorFlag(PxActorFlag::eVISUALIZATION, true);
	box->setAngularDamping(0.5f);
	box->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	m_scene->addActor(*box);
	if (linVel)
		box->setLinearVelocity(*(PxVec3*)linVel);
	return box;
}


physx::PxRigidDynamic* cPhysicsEngine::CreateSphere(const Vector3& pos
	, const float radius
	, const Vector3* linVel // = nullptr
	, float density // = 1.f
)
{
	PxSceneWriteLock scopedLock(*m_scene);
	PxRigidDynamic* sphere = PxCreateDynamic(*m_physics, PxTransform(*(PxVec3*)&pos)
		, PxSphereGeometry(radius), *m_material, density);
	PX_ASSERT(sphere);

	sphere->setActorFlag(PxActorFlag::eVISUALIZATION, true);
	sphere->setAngularDamping(0.5f);
	sphere->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	m_scene->addActor(*sphere);

	if (linVel)
		sphere->setLinearVelocity(*(PxVec3*)linVel);

	return sphere;
}


physx::PxRigidDynamic* cPhysicsEngine::CreateCapsule(const Vector3& pos
	, const float radius, const float halfHeight
	, const Vector3* linVel //= nullptr
	, float density //= 1.f
)
{
	PxSceneWriteLock scopedLock(*m_scene);
	const PxQuat rot = PxQuat(PxIdentity);
	PX_UNUSED(rot);

	PxRigidDynamic* capsule = PxCreateDynamic(*m_physics, PxTransform(*(PxVec3*)&pos)
		, PxCapsuleGeometry(radius, halfHeight), *m_material, density);
	PX_ASSERT(capsule);

	capsule->setActorFlag(PxActorFlag::eVISUALIZATION, true);
	capsule->setAngularDamping(0.5f);
	capsule->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	m_scene->addActor(*capsule);

	if (linVel)
		capsule->setLinearVelocity(*(PxVec3*)linVel);

	return capsule;
}


// PxDeletionListener interface
void cPhysicsEngine::onRelease(const physx::PxBase* observed, void* userData
	, physx::PxDeletionEventFlag::Enum deletionEvent)
{
	// nothing~
}



cSphere* cPhysicsEngine::SpawnSphere(graphic::cRenderer &renderer
	, const Vector3& pos, const float radius)
{
	cSphere *sphere = new cSphere();
	sphere->Create(renderer, radius, 10, 10);
	sphere->m_transform.pos = pos;
	return sphere;
}


cCube* cPhysicsEngine::SpawnBox(graphic::cRenderer &renderer
	, const Vector3& pos, const Vector3 &scale)
{
	cCube *cube = new cCube();
	cube->Create(renderer);
	Transform tfm;
	tfm.pos = pos;
	tfm.scale = scale;
	cube->SetCube(tfm);
	return cube;
}


cCapsule* cPhysicsEngine::SpawnCapsule(graphic::cRenderer &renderer
	, const Vector3& pos, const float radius, const float halfHeight)
{
	cCapsule *capsule = new cCapsule();
	capsule->Create(renderer, radius, halfHeight, 16, 8);
	capsule->SetPos(pos);
	return capsule;
}


bool cPhysicsEngine::AddJoint(physx::PxJoint *joint)
{
	if (!joint)
		return false;

	m_joints.push_back(joint);
	return true;
}


void cPhysicsEngine::ClearPhysicsObject()
{
	// no remove ground plane object
	sActor ground;
	for (auto &p : m_actors)
	{
		if (p.name == "grid")
		{
			ground = p;
			continue;
		}
		m_scene->removeActor(*p.actor);
		PX_SAFE_RELEASE(p.actor);
		SAFE_DELETE(p.node);
	}
	m_actors.clear();
	if (ground.name == "grid")
		m_actors.push_back(ground);

	for (auto &j : m_joints)
		PX_SAFE_RELEASE(j);
	m_joints.clear();
}
