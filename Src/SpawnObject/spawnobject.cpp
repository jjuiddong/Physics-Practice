//
// PhysX SpawnObject Sample
//	- reference
//		- PhysXSample code
// 
// 2025-01-19
//	- migration physx3.4 -> 5.5
//
#include "../../../Common/Common/common.h"
using namespace common;
#include "../../../Common/Graphic11/graphic11.h"
#include "../../../Common/Framework11/framework11.h"

// physx release macro
#if !defined(PX_SAFE_RELEASE)
	#define PX_SAFE_RELEASE(p) {if((p)) {p->release(); p = nullptr;}}
#endif

#include "PxPhysicsAPI.h"

#if defined(_WIN64)
#if defined(_DEBUG)
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/debug/PhysX_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/debug/PhysXCommon_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/debug/PhysXCooking_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/debug/PhysXFoundation_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/debug/PhysXExtensions_static_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/debug/PhysXPvdSDK_static_64.lib")
#else
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/release/PhysX_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/release/PhysXCommon_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/release/PhysXCooking_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/release/PhysXFoundation_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/release/PhysXExtensions_static_64.lib")
#pragma comment(lib, "../External/PhysX-5.5/bin/win.x86_64.vc142.md/release/PhysXPvdSDK_static_64.lib")
#endif
#endif

using namespace graphic;
using namespace framework;

class cDefaultErrorCallback : public physx::PxErrorCallback
{
public:
	virtual void reportError(physx::PxErrorCode::Enum code, const char* message
		, const char* file, int line)
	{
		int a = 0;
	}
};

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

struct sSyncInfo
{
	physx::PxRigidActor* actor;
	string name;
	cNode* node;
};


class cViewer : public framework::cGameMain2
	, public physx::PxDeletionListener
{
public:
	cViewer();
	virtual ~cViewer();
	virtual bool OnInit() override;
	virtual void OnUpdate(const float deltaSeconds) override;
	virtual void OnRender(const float deltaSeconds) override;
	virtual void OnEventProc(const sf::Event &evt) override;

protected:
	bool InitializePhysx();

	physx::PxRigidActor* CreateGrid();
	physx::PxRigidDynamic* CreateBox(const Vector3& pos
		, const Vector3& dims, const Vector3* linVel
		, float density, sSyncInfo* sync);
	physx::PxRigidDynamic* CreateSphere(const Vector3& pos
		, const float radius, const Vector3* linVel, float density 
		, sSyncInfo* sync);
	physx::PxRigidDynamic* CreateCapsule(const Vector3& pos
		, const float radius, const float halfHeight, const Vector3* linVel
		, float density, sSyncInfo* sync);

	cCube* SpawnBox(const Vector3& pos, const Vector3 &scale);
	cSphere* SpawnSphere(const Vector3& pos, const float radius);
	cCapsule* SpawnCapsule(const Vector3& pos, const float radius, const float halfHeight);

	// PxDeletionListener override
	virtual void onRelease(const physx::PxBase* observed, void* userData
		, physx::PxDeletionEventFlag::Enum deletionEvent) override;


public:
	physx::PxFoundation *m_foundation;
	physx::PxPhysics *m_physics;
	physx::PxMaterial *m_material;
	physx::PxPvd *m_pvd;
	physx::PxPvdTransport *m_transport;
	physx::PxDefaultAllocator m_defaultAllocatorCallback;
	physx::PxPvdInstrumentationFlags m_pvdFlags;
	cDefaultErrorCallback m_defaultErrorCallback;
	physx::PxDefaultCpuDispatcher *m_cpuDispatcher;
	physx::PxCudaContextManager *m_cudaContextManager;
	physx::PxScene *m_scene;

	vector<sSyncInfo*> m_syncs;
};

INIT_FRAMEWORK3(cViewer);


cViewer::cViewer()
{
	m_windowName = L"Spawn Object";
	m_isLazyMode = true;
	const RECT r = { 0, 0, 1024, 768 };
	//const RECT r = { 0, 0, 1280, 960 };
	m_windowRect = r;
	graphic::cResourceManager::Get()->SetMediaDirectory("./media/");
}

cViewer::~cViewer()
{
	for (auto &p : m_syncs)
	{
		m_scene->removeActor(*p->actor);
		PX_SAFE_RELEASE(p->actor);
		SAFE_DELETE(p->node);
		delete p;
	}
	m_syncs.clear();

	m_physics->unregisterDeletionListener(*this);
	PX_SAFE_RELEASE(m_scene);
	PX_SAFE_RELEASE(m_cudaContextManager);
	PX_SAFE_RELEASE(m_cpuDispatcher);
	PX_SAFE_RELEASE(m_material);
	PX_SAFE_RELEASE(m_physics);
	PX_SAFE_RELEASE(m_pvd);
	PX_SAFE_RELEASE(m_transport);
	PX_SAFE_RELEASE(m_foundation);
}


bool cViewer::OnInit()
{
	dbg::RemoveLog();
	dbg::RemoveErrLog();

	const float WINSIZE_X = float(m_windowRect.right - m_windowRect.left);
	const float WINSIZE_Y = float(m_windowRect.bottom - m_windowRect.top);
	GetMainCamera().SetCamera(Vector3(10.0f, 10, -30.f), Vector3(-5, 0, 10), Vector3(0, 1, 0));
	GetMainCamera().SetProjection(MATH_PI / 4.f, (float)WINSIZE_X / (float)WINSIZE_Y, 1.f, 10000.f);
	GetMainCamera().SetViewPort(WINSIZE_X, WINSIZE_Y);

	GetMainLight().Init(cLight::LIGHT_DIRECTIONAL,
		Vector4(0.2f, 0.2f, 0.2f, 1), Vector4(0.9f, 0.9f, 0.9f, 1),
		Vector4(0.2f, 0.2f, 0.2f, 1));
	const Vector3 lightPos(-300, 300, -300);
	const Vector3 lightLookat(0, 0, 0);
	GetMainLight().SetPosition(lightPos);
	GetMainLight().SetDirection((lightLookat - lightPos).Normal());

	if (!InitializePhysx())
		return false;

	cGridLine *gridLine = new cGridLine();
	gridLine->Create(m_renderer, 100, 100, 1.f, 1.f);
	sSyncInfo* sync1 = new sSyncInfo;
	sync1->actor = CreateGrid();
	sync1->name = "grid";
	sync1->node = gridLine;
	m_syncs.push_back(sync1);

	const Vector3 pos(0, 10, 0);
	const float scale = 0.5f;
	
	cSphere *sphere = SpawnSphere(pos, 0.5f);
	sSyncInfo* sync2 = new sSyncInfo;
	sync2->actor = CreateSphere(pos, scale, nullptr, 1.f, sync2);
	sync2->name = "sphere";
	sync2->node = sphere;
	m_syncs.push_back(sync2);

	m_gui.SetContext();
	m_gui.SetStyleColorsDark();

	return true;
}


bool cViewer::InitializePhysx()
{
	m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION
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
	//if (!sceneDesc.gpuDispatcher && m_cudaContextManager)
	//	sceneDesc.gpuDispatcher = m_cudaContextManager->getGpuDispatcher();

	//sceneDesc.frictionType = physx::PxFrictionType::eTWO_DIRECTIONAL;
	//sceneDesc.frictionType = physx::PxFrictionType::eONE_DIRECTIONAL;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_PCM;
	//sceneDesc.flags |= physx::PxSceneFlag::eENABLE_AVERAGE_POINT;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_STABILIZATION;
	//sceneDesc.flags |= physx::PxSceneFlag::eADAPTIVE_FORCE;
	//sceneDesc.flags |= physx::PxSceneFlag::eENABLE_ACTIVETRANSFORMS; // Physx 3.4
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS; // Physx 4.0
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


// PxDeletionListener interface
void cViewer::onRelease(const physx::PxBase* observed, void* userData
	, physx::PxDeletionEventFlag::Enum deletionEvent)
{
	// nothing~
}


void cViewer::OnUpdate(const float deltaSeconds)
{
	__super::OnUpdate(deltaSeconds);
	GetMainCamera().Update(deltaSeconds);

	const float step = 1.f / 60.f;
	static float dt = 0.f;
	dt += deltaSeconds;
	if (dt > step)
	{
		dt -= step;
		physx::PxSceneWriteLock writeLock(*m_scene);
		m_scene->simulate(step, nullptr);
	}
}


void cViewer::OnRender(const float deltaSeconds)
{
	GetMainCamera().Bind(m_renderer);
	GetMainLight().Bind(m_renderer);

	for (auto &sync : m_syncs)
		sync->node->Render(m_renderer);

	{
		using namespace physx;

		// update physx result info
		{
			PxSceneWriteLock writeLock(*m_scene);
			m_scene->fetchResults(true);
		}

		uint size = 0;
		{
			PxActor** activeActors = m_scene->getActiveActors(size);

			for (uint i = 0; i < size; ++i)
			{
				sSyncInfo* sync = (sSyncInfo*)activeActors[i]->userData;
				if (sync && sync->actor)
				{
					const PxTransform gtm = sync->actor->getGlobalPose();

					Transform tfm;
					tfm.pos = *(Vector3*)&gtm.p;
					tfm.rot = *(Quaternion*)&gtm.q;

					tfm.scale = sync->node->m_transform.scale;
					sync->node->m_transform = tfm;
				}
			}
		}
	}
}


physx::PxRigidActor* cViewer::CreateGrid()
{
	using namespace physx;
	PxSceneWriteLock scopedLock(*m_scene);

	PxRigidStatic* plane = PxCreatePlane(*m_physics
		, PxPlane(PxVec3(0, 1, 0), 0), *m_material);
	if (!plane)
		return nullptr;

	m_scene->addActor(*plane);

	PxShape* shape;
	plane->getShapes(&shape, 1);
	return plane;
}


physx::PxRigidDynamic* cViewer::CreateBox(const Vector3& pos
	, const Vector3& dims
	, const Vector3* linVel // = nullptr
	, float density
	, sSyncInfo* sync
)
{
	using namespace physx;
	PxSceneWriteLock scopedLock(*m_scene);
	PxRigidDynamic* box = PxCreateDynamic(*m_physics, PxTransform(*(PxVec3*)&pos)
		, PxBoxGeometry(*(PxVec3*)&dims), *m_material, density);
	PX_ASSERT(box);

	box->setActorFlag(PxActorFlag::eVISUALIZATION, true);
	box->setAngularDamping(0.5f);
	box->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	box->userData = sync;
	m_scene->addActor(*box);
	if (linVel)
		box->setLinearVelocity(*(PxVec3*)linVel);
	return box;
}


physx::PxRigidDynamic* cViewer::CreateSphere(const Vector3& pos
	, const float radius
	, const Vector3* linVel // = nullptr
	, float density
	, sSyncInfo* sync
)
{
	using namespace physx;
	PxSceneWriteLock scopedLock(*m_scene);
	PxRigidDynamic* sphere = PxCreateDynamic(*m_physics, PxTransform(*(PxVec3*)&pos)
		, PxSphereGeometry(radius), *m_material, density);
	PX_ASSERT(sphere);

	sphere->setActorFlag(PxActorFlag::eVISUALIZATION, true);
	sphere->setAngularDamping(0.5f);
	sphere->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	sphere->userData = sync;
	m_scene->addActor(*sphere);

	if (linVel)
		sphere->setLinearVelocity(*(PxVec3*)linVel);

	return sphere;
}


physx::PxRigidDynamic* cViewer::CreateCapsule(const Vector3& pos
	, const float radius, const float halfHeight
	, const Vector3* linVel //= nullptr
	, float density
	, sSyncInfo* sync
)
{
	using namespace physx;
	PxSceneWriteLock scopedLock(*m_scene);
	const PxQuat rot = PxQuat(PxIdentity);
	PX_UNUSED(rot);

	PxRigidDynamic* capsule = PxCreateDynamic(*m_physics, PxTransform(*(PxVec3*)&pos)
		, PxCapsuleGeometry(radius, halfHeight), *m_material, density);
	PX_ASSERT(capsule);

	capsule->setActorFlag(PxActorFlag::eVISUALIZATION, true);
	capsule->setAngularDamping(0.5f);
	capsule->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	capsule->userData = sync;
	m_scene->addActor(*capsule);

	if (linVel)
		capsule->setLinearVelocity(*(PxVec3*)linVel);

	return capsule;
}


cSphere* cViewer::SpawnSphere(const Vector3& pos, const float radius)
{
	cSphere *sphere = new cSphere();
	sphere->Create(m_renderer, radius, 10, 10);
	sphere->m_transform.pos = pos;
	return sphere;
}


cCube* cViewer::SpawnBox(const Vector3& pos, const Vector3 &scale)
{
	cCube *cube = new cCube();
	cube->Create(m_renderer);
	Transform tfm;
	tfm.pos = pos;
	tfm.scale = scale;
	cube->SetCube(tfm);
	return cube;
}


cCapsule* cViewer::SpawnCapsule(const Vector3& pos, const float radius, const float halfHeight)
{
	cCapsule *capsule = new cCapsule();
	capsule->Create(m_renderer, radius, halfHeight, 16, 8);
	capsule->SetPos(pos);
	return capsule;
}


void cViewer::OnEventProc(const sf::Event &evt)
{
	ImGuiIO& io = ImGui::GetIO();
	switch (evt.type)
	{
	case sf::Event::KeyPressed:
		switch (evt.key.cmd) {
		case sf::Keyboard::Escape: close(); break;
		case sf::Keyboard::Space:
		{
			const Vector3 pos(0, 10, 0);
			const float scale = 0.5f;
			static int idx = 2;
			cNode *node = nullptr;
			switch (idx)
			{
			case 0: 
			{
				node = SpawnBox(pos, Vector3::Ones * scale);

				sSyncInfo* sync = new sSyncInfo;
				sync->actor = CreateBox(pos, Vector3::Ones * scale, nullptr, 1.f, sync);
				sync->name = "cube";
				sync->node = node;
				m_syncs.push_back(sync);
			}
			break;

			case 1: 
			{
				node = SpawnSphere(pos, scale);

				sSyncInfo* sync = new sSyncInfo;
				sync->actor = CreateSphere(pos, scale, nullptr, 1.f, sync);
				sync->name = "sphere";
				sync->node = node;
				m_syncs.push_back(sync);
			}
			break;

			case 2:
			{
				node = SpawnCapsule(pos, scale, scale*2.f);

				sSyncInfo* sync = new sSyncInfo;
				sync->actor = CreateCapsule(pos, scale, scale*2.f, nullptr, 1.f, sync);
				sync->name = "capsule";
				sync->node = node;
				m_syncs.push_back(sync);
			}
			break;
			}
			
			++idx;
			idx %= 3;
		}
		break;
		}
		break;
	}
}
