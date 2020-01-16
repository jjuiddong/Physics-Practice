//
// PhysX Simple Sample
//	- reference
//		- PhysXSample code
//
#include "stdafx.h"
#include "PxPhysicsAPI.h"

#if defined(_DEBUG)
	#pragma comment(lib, "../../../PhysX-3.4/PhysX_3.4/Lib/vc15win32/PhysX3DEBUG_x86.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PhysX_3.4/lib/vc15win32/PhysX3CookingDEBUG_x86.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PhysX_3.4/lib/vc15win32/PhysX3CommonDEBUG_x86.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PhysX_3.4/lib/vc15win32/PhysX3ExtensionsDEBUG.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PxShared/lib/vc15win32/PxFoundationDEBUG_x86.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PxShared/lib/vc15win32/PxPvdSDKDEBUG_x86.lib")
#else
	#pragma comment(lib, "../../../PhysX-3.4/PhysX_3.4/Lib/vc15win32/PhysX3_x86.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PhysX_3.4/lib/vc15win32/PhysX3Cooking_x86.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PhysX_3.4/lib/vc15win32/PhysX3Common_x86.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PhysX_3.4/lib/vc15win32/PhysX3Extensions.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PxShared/lib/vc15win32/PxFoundation_x86.lib")
	#pragma comment(lib, "../../../PhysX-3.4/PxShared/lib/vc15win32/PxPvdSDK_x86.lib")
#endif

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

using namespace graphic;
using namespace framework;

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
		, float density);

	// PxDeletionListener override
	virtual void onRelease(const physx::PxBase* observed, void* userData
		, physx::PxDeletionEventFlag::Enum deletionEvent) override;


public:
	physx::PxFoundation *m_foundation;
	physx::PxPhysics *m_physics;
	physx::PxMaterial *m_material;
	physx::PxPvd *m_pvd;
	physx::PxPvdTransport *m_transport;
	physx::PxCooking *m_cooking;
	physx::PxDefaultAllocator m_defaultAllocatorCallback;
	physx::PxPvdInstrumentationFlags m_pvdFlags;
	cDefaultErrorCallback m_defaultErrorCallback;
	physx::PxDefaultCpuDispatcher *m_cpuDispatcher;
	physx::PxCudaContextManager *m_cudaContextManager;
	physx::PxScene *m_scene;
	uint m_activeBufferCapacity;
	physx::PxActiveTransform *m_bufferedActiveTransforms;

	struct sActor {
		physx::PxRigidActor *actor;
		string name;
		cNode *node;
	};
	vector<sActor> m_actors;
};

INIT_FRAMEWORK3(cViewer);


cViewer::cViewer()
{
	m_windowName = L"Simple";
	m_isLazyMode = true;
	const RECT r = { 0, 0, 1024, 768 };
	//const RECT r = { 0, 0, 1280, 960 };
	m_windowRect = r;
	graphic::cResourceManager::Get()->SetMediaDirectory("./media/");
}

cViewer::~cViewer()
{
	for (auto &p : m_actors)
	{
		m_scene->removeActor(*p.actor);
		PX_SAFE_RELEASE(p.actor);
		SAFE_DELETE(p.node);
	}

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

	cCube *cube = new cCube();
	cube->Create(m_renderer);
	const Vector3 boxPos(0, 10, 0);
	const float boxScale = 0.5f;
	Transform tfm;
	tfm.pos = boxPos;
	tfm.scale = Vector3(1, 1, 1) * boxScale;
	cube->SetCube(tfm);

	m_actors.push_back({ CreateGrid(), "grid", gridLine });
	m_actors.push_back({ CreateBox(boxPos, Vector3::Ones*boxScale, nullptr, 1.f), "cube", cube });

	m_activeBufferCapacity = m_actors.size();
	m_bufferedActiveTransforms = (physx::PxActiveTransform*)_aligned_malloc(
		sizeof(physx::PxActiveTransform) * m_actors.size(), 16);

	m_gui.SetContext();
	m_gui.SetStyleColorsDark();

	return true;
}


bool cViewer::InitializePhysx()
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

	physx::PxSceneWriteLock writeLock(*m_scene);
	m_scene->simulate(0.01f, nullptr);
}


void cViewer::OnRender(const float deltaSeconds)
{
	GetMainCamera().Bind(m_renderer);
	GetMainLight().Bind(m_renderer);

	for (auto &actor : m_actors)
		actor.node->Render(m_renderer);

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
				return (a.actor == activeTfm->actor);});
			if (m_actors.end() == it)
				continue;

			sActor &actor = *it;
			if (actor.name == "cube")
			{
				Transform tfm = actor.node->m_transform;
				tfm.pos = *(Vector3*)&activeTfm->actor2World.p;
				actor.node->m_transform = tfm;
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
	, const Vector3& dims, const Vector3* linVel
	, float density)
{
	using namespace physx;
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
			cCube *cube = new cCube();
			cube->Create(m_renderer);
			const Vector3 boxPos(0, 10, 0);
			const float boxScale = 0.5f;
			Transform tfm;
			tfm.pos = boxPos;
			tfm.scale = Vector3(1, 1, 1) * boxScale;
			cube->SetCube(tfm);

			m_actors.push_back({ CreateBox(Vector3(0,10,0)
				, Vector3::Ones*boxScale, nullptr, 1.f), "cube", cube });
		}
		break;
		}
		break;
	}
}
