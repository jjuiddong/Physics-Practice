//
// 2020-01-16, jjuiddong
// Information View
//
#pragma once


class cInformationView : public framework::cDockWindow
{
public:
	cInformationView(const StrId &name);
	virtual ~cInformationView();

	virtual void OnUpdate(const float deltaSeconds) override;
	virtual void OnRender(const float deltaSeconds) override;


protected:
	void RenderActorEdit();
	void RenderFixedJoint();
	void RenderSphericalJoint();
	void RenderRevoluteJoint();
	void RenderPrismaticJoint();
	void RenderDistanceJoint();
	void RenderD6Joint();

	struct sSpawnObj {
		graphic::cNode *node0;
		graphic::cNode *node1;
		physx::PxRigidDynamic *actor0;
		physx::PxRigidDynamic *actor1;
	};
	sSpawnObj SpawnObject();


public:
	float m_distance;
	float m_radius;
	int m_item;
	bool m_isKinematic0;
	bool m_isKinematic1;
	bool m_isLimitJoint;
	Vector3 m_pos0;
	Vector3 m_scale0;
	Vector3 m_pos1;
	Vector3 m_scale1;
};
