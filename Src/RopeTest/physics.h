//
// 2020-01-16, jjuiddong
// PhysX library wrapping module
//
// 2025-01-19
//	- migration physx3.4 -> 5.5
//
#pragma once


class cDefaultErrorCallback : public physx::PxErrorCallback
{
public:
	virtual void reportError(physx::PxErrorCode::Enum code, const char* message
		, const char* file, int line) {
		int a = 0;
	}
};

struct sSyncInfo
{
	physx::PxRigidActor* actor;
	string name;
	graphic::cNode* node;
};


class cPhysicsEngine : public physx::PxDeletionListener
{
public:
	cPhysicsEngine();
	virtual ~cPhysicsEngine();

	bool InitializePhysx(graphic::cRenderer &renderer);
	bool PreUpdate(const float deltaSeconds);
	bool PostUpdate(const float deltaSeconds);

	physx::PxRigidActor* CreateGrid();
	physx::PxRigidDynamic* CreateBox(const Vector3& pos
		, const Vector3& dims, const Vector3* linVel = nullptr
		, float density = 1.f);
	physx::PxRigidDynamic* CreateSphere(const Vector3& pos
		, const float radius, const Vector3* linVel = nullptr, float density = 1.f);
	physx::PxRigidDynamic* CreateCapsule(const Vector3& pos
		, const float radius, const float halfHeight, const Vector3* linVel = nullptr
		, float density = 1.f);

	graphic::cCube* SpawnBox(graphic::cRenderer &renderer
		, const Vector3& pos, const Vector3 &scale);
	graphic::cSphere* SpawnSphere(graphic::cRenderer &renderer
		, const Vector3& pos, const float radius);
	graphic::cCapsule* SpawnCapsule(graphic::cRenderer &renderer
		, const Vector3& pos, const float radius, const float halfHeight);

	bool AddJoint(physx::PxJoint *joint);
	void ClearPhysicsObject();


protected:
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
	vector<physx::PxJoint*> m_joints;
};
