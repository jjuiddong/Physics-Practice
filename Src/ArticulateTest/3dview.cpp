
#include "stdafx.h"
#include "3dview.h"

using namespace graphic;
using namespace framework;


c3DView::c3DView(const string &name)
	: framework::cDockWindow(name)
	, m_showGrid(true)
	, m_isSimulation(false)
{
}

c3DView::~c3DView()
{
	// save config variable
	const Vector3 eyePos = m_camera.m_eyePos;
	const Vector3 lookAt = m_camera.m_lookAt;
	common::cConfig config;
	stringstream ss1;
	ss1 << eyePos.x << ", " << eyePos.y << ", " << eyePos.z;
	config.SetValue("eyepos", ss1.str().c_str());
	stringstream ss2;
	ss2 << lookAt.x << ", " << lookAt.y << ", " << lookAt.z;
	config.SetValue("lookat", ss2.str().c_str());
	config.Write("config.txt");
	//~

	m_physics.Clear();
}


bool c3DView::Init(cRenderer& renderer)
{
	Vector3 eyePos(26.5719681f, 44.7804565f, -53.4074707f);
	Vector3 lookAt(0.0922393799f, 0.00000000f, 2.20689774f);
	common::cConfig config;
	if (config.Read("config.txt"))
	{
		eyePos = config.GetVector3("eyepos", eyePos);
		lookAt = config.GetVector3("lookat", lookAt);
	}

	m_camera.SetCamera(eyePos, lookAt, Vector3(0, 1, 0));
	m_camera.SetProjection(MATH_PI / 4.f, m_rect.Width() / m_rect.Height(), 1.f, 1000000.f);
	m_camera.SetViewPort(m_rect.Width(), m_rect.Height());

	GetMainLight().Init(graphic::cLight::LIGHT_DIRECTIONAL);
	GetMainLight().SetDirection(Vector3(-1, -2, 1.3f).Normal());

	sf::Vector2u size((uint)m_rect.Width() - 15, (uint)m_rect.Height() - 50);
	cViewport vp = renderer.m_viewPort;
	vp.m_vp.Width = (float)size.x;
	vp.m_vp.Height = (float)size.y;
	m_renderTarget.Create(renderer, vp, DXGI_FORMAT_R8G8B8A8_UNORM, true, true
		, DXGI_FORMAT_D24_UNORM_S8_UINT);
	m_grid.Create(renderer, 200, 200, 1.f, 1.f
		, (eVertexType::POSITION | eVertexType::COLOR)
		, cColor(0.6f, 0.6f, 0.6f, 0.5f)
		, cColor(0.f, 0.f, 0.f, 0.5f)
	);

	if (!m_physics.InitializePhysx())
		return false;

	m_physSync = new phys::cPhysicsSync();
	m_physSync->Create(&m_physics);
	m_physSync->SpawnPlane(renderer, Vector3(0, 1, 0));

	//InitScissorLift1();
	//InitScissorLift2();
	InitScissorLift3();

	
	//m_boxId = m_physSync->SpawnBox(renderer, Transform(Vector3(0, 20, 0), Vector3::Ones*0.5f));
	m_box.Create(renderer);
	m_box.SetCube(Transform(Vector3(0, 0, 0), Vector3::Ones*0.5f));

	return true;
}


// initialize scissor lift1
void c3DView::InitScissorLift1()
{
	using namespace physx;


	const PxReal runnerLength = 2.f;
	const PxReal placementDistance = 1.8f;

	const PxReal cosAng = (placementDistance) / (runnerLength);

	const PxReal angle = PxAcos(cosAng);

	const PxReal sinAng = PxSin(angle);

	const PxQuat leftRot(-angle, PxVec3(1.f, 0.f, 0.f));
	const PxQuat rightRot(angle, PxVec3(1.f, 0.f, 0.f));

	m_articulation = m_physics.m_physics->createArticulationReducedCoordinate();
	m_articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);

	//(1) Create base...
	PxArticulationLink* base = m_articulation->createLink(NULL, PxTransform(PxVec3(0.f, 0.25f, 0.f)));
	PxRigidActorExt::createExclusiveShape(*base, PxBoxGeometry(0.5f, 0.25f, 1.5f), *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*base, 3.f);

	//Now create the slider and fixed joints...

	m_articulation->setSolverIterationCounts(32);

	PxArticulationLink* leftRoot = m_articulation->createLink(base, PxTransform(PxVec3(0.f, 0.55f, -0.9f)));
	PxRigidActorExt::createExclusiveShape(*leftRoot, PxBoxGeometry(0.5f, 0.05f, 0.05f), *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*leftRoot, 1.f);

	PxArticulationLink* rightRoot = m_articulation->createLink(base, PxTransform(PxVec3(0.f, 0.55f, 0.9f)));
	PxRigidActorExt::createExclusiveShape(*rightRoot, PxBoxGeometry(0.5f, 0.05f, 0.05f), *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*rightRoot, 1.f);

	PxArticulationJointReducedCoordinate* joint = leftRoot->getInboundJoint();
	joint->setJointType(PxArticulationJointType::eFIX);
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.25f, -0.9f)));
	joint->setChildPose(PxTransform(PxVec3(0.f, -0.05f, 0.f)));

	m_driveJoint = rightRoot->getInboundJoint();
	m_driveJoint->setJointType(PxArticulationJointType::ePRISMATIC);
	m_driveJoint->setMotion(PxArticulationAxis::eZ, PxArticulationMotion::eLIMITED);
	m_driveJoint->setLimitParams(PxArticulationAxis::eZ, PxArticulationLimit(-1.4f, 0.2f));
	m_driveJoint->setDriveParams(PxArticulationAxis::eZ, PxArticulationDrive(100000.f, 0.f, PX_MAX_F32));

	m_driveJoint->setParentPose(PxTransform(PxVec3(0.f, 0.25f, 0.9f)));
	m_driveJoint->setChildPose(PxTransform(PxVec3(0.f, -0.05f, 0.f)));

	const PxU32 linkHeight = 3;
	PxArticulationLink* currLeft = leftRoot, * currRight = rightRoot;

	PxQuat rightParentRot(PxIdentity);
	PxQuat leftParentRot(PxIdentity);
	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		const PxVec3 pos(0.5f, 0.55f + 0.1f * (1 + i), 0.f);
		PxArticulationLink* leftLink = m_articulation->createLink(currLeft, PxTransform(pos + PxVec3(0.f, sinAng * (2 * i + 1), 0.f), leftRot));
		PxRigidActorExt::createExclusiveShape(*leftLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *m_physics.m_material);
		PxRigidBodyExt::updateMassAndInertia(*leftLink, 1.f);

		const PxVec3 leftAnchorLocation = pos + PxVec3(0.f, sinAng * (2 * i), -0.9f);

		joint = leftLink->getInboundJoint();
		joint->setParentPose(PxTransform(currLeft->getGlobalPose().transformInv(leftAnchorLocation), leftParentRot));
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, -1.f), rightRot));
		joint->setJointType(PxArticulationJointType::eREVOLUTE);

		leftParentRot = leftRot;

		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-PxPi, angle));


		PxArticulationLink* rightLink = m_articulation->createLink(currRight, PxTransform(pos + PxVec3(0.f, sinAng * (2 * i + 1), 0.f), rightRot));
		PxRigidActorExt::createExclusiveShape(*rightLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *m_physics.m_material);
		PxRigidBodyExt::updateMassAndInertia(*rightLink, 1.f);

		const PxVec3 rightAnchorLocation = pos + PxVec3(0.f, sinAng * (2 * i), 0.9f);

		joint = rightLink->getInboundJoint();
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setParentPose(PxTransform(currRight->getGlobalPose().transformInv(rightAnchorLocation), rightParentRot));
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 1.f), leftRot));
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-angle, PxPi));

		rightParentRot = rightRot;

		PxD6Joint* d6joint = PxD6JointCreate(*m_physics.m_physics, leftLink, PxTransform(PxIdentity), rightLink, PxTransform(PxIdentity));

		d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currLeft = rightLink;
		currRight = leftLink;
	}


	PxArticulationLink* leftTop = m_articulation->createLink(currLeft, currLeft->getGlobalPose().transform(PxTransform(PxVec3(-0.5f, 0.f, -1.0f), leftParentRot)));
	PxRigidActorExt::createExclusiveShape(*leftTop, PxBoxGeometry(0.5f, 0.05f, 0.05f), *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*leftTop, 1.f);

	PxArticulationLink* rightTop = m_articulation->createLink(currRight, currRight->getGlobalPose().transform(PxTransform(PxVec3(-0.5f, 0.f, 1.0f), rightParentRot)));
	//PxRigidActorExt::createExclusiveShape(*rightTop, PxCapsuleGeometry(0.05f, 0.8f), *m_physics.m_material);
	PxRigidActorExt::createExclusiveShape(*rightTop, PxBoxGeometry(0.5f, 0.05f, 0.05f), *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*rightTop, 1.f);

	joint = leftTop->getInboundJoint();
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.f, -1.f), currLeft->getGlobalPose().q.getConjugate()));
	joint->setChildPose(PxTransform(PxVec3(0.5f, 0.f, 0.f), leftTop->getGlobalPose().q.getConjugate()));
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);

	joint = rightTop->getInboundJoint();
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.f, 1.f), currRight->getGlobalPose().q.getConjugate()));
	joint->setChildPose(PxTransform(PxVec3(0.5f, 0.f, 0.f), rightTop->getGlobalPose().q.getConjugate()));
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);


	currLeft = leftRoot;
	currRight = rightRoot;

	rightParentRot = PxQuat(PxIdentity);
	leftParentRot = PxQuat(PxIdentity);


	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		const PxVec3 pos(-0.5f, 0.55f + 0.1f * (1 + i), 0.f);
		PxArticulationLink* leftLink = m_articulation->createLink(currLeft, PxTransform(pos + PxVec3(0.f, sinAng * (2 * i + 1), 0.f), leftRot));
		PxRigidActorExt::createExclusiveShape(*leftLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *m_physics.m_material);
		PxRigidBodyExt::updateMassAndInertia(*leftLink, 1.f);

		const PxVec3 leftAnchorLocation = pos + PxVec3(0.f, sinAng * (2 * i), -0.9f);

		joint = leftLink->getInboundJoint();
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setParentPose(PxTransform(currLeft->getGlobalPose().transformInv(leftAnchorLocation), leftParentRot));
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, -1.f), rightRot));

		leftParentRot = leftRot;

		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-PxPi, angle));

		PxArticulationLink* rightLink = m_articulation->createLink(currRight, PxTransform(pos + PxVec3(0.f, sinAng * (2 * i + 1), 0.f), rightRot));
		PxRigidActorExt::createExclusiveShape(*rightLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *m_physics.m_material);
		PxRigidBodyExt::updateMassAndInertia(*rightLink, 1.f);

		const PxVec3 rightAnchorLocation = pos + PxVec3(0.f, sinAng * (2 * i), 0.9f);

		/*joint = PxD6JointCreate(getPhysics(), currRight, PxTransform(currRight->getGlobalPose().transformInv(rightAnchorLocation)),
		rightLink, PxTransform(PxVec3(0.f, 0.f, 1.f)));*/

		joint = rightLink->getInboundJoint();
		joint->setParentPose(PxTransform(currRight->getGlobalPose().transformInv(rightAnchorLocation), rightParentRot));
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 1.f), leftRot));
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-angle, PxPi));

		rightParentRot = rightRot;

		PxD6Joint* d6joint = PxD6JointCreate(*m_physics.m_physics, leftLink, PxTransform(PxIdentity), rightLink, PxTransform(PxIdentity));

		d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currLeft = rightLink;
		currRight = leftLink;
	}

	PxD6Joint* d6joint = PxD6JointCreate(*m_physics.m_physics, currLeft, PxTransform(PxVec3(0.f, 0.f, -1.f)), leftTop, PxTransform(PxVec3(-0.5f, 0.f, 0.f)));

	d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

	d6joint = PxD6JointCreate(*m_physics.m_physics, currRight, PxTransform(PxVec3(0.f, 0.f, 1.f)), rightTop, PxTransform(PxVec3(-0.5f, 0.f, 0.f)));

	d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);


	const PxTransform topPose(PxVec3(0.f, leftTop->getGlobalPose().p.y + 0.15f, 0.f));

	PxArticulationLink* top = m_articulation->createLink(leftTop, topPose);
	PxRigidActorExt::createExclusiveShape(*top, PxBoxGeometry(0.5f, 0.1f, 1.5f), *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*top, 1.f);

	joint = top->getInboundJoint();
	joint->setJointType(PxArticulationJointType::eFIX);
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.0f, 0.f)));
	joint->setChildPose(PxTransform(PxVec3(0.f, -0.15f, -0.9f)));


	for (PxU32 i = 0; i < m_articulation->getNbLinks(); ++i)
	{
		PxArticulationLink* link;
		m_articulation->getLinks(&link, 1, i);

		link->setLinearDamping(0.2f);
		link->setAngularDamping(0.2f);

		link->setMaxAngularVelocity(20.f);
		link->setMaxLinearVelocity(100.f);

		if (link != top)
		{
			for (PxU32 b = 0; b < link->getNbShapes(); ++b)
			{
				PxShape* shape;
				link->getShapes(&shape, 1, b);

				shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));
			}
		}
	}

	m_physics.m_scene->addArticulation(*m_articulation);
}


// create articulation link
physx::PxArticulationLink* c3DView::CreateLink(physx::PxArticulationLink* parent
	, const Transform& tfm
	, const float mass //= 1.f
)
{
	using namespace physx;

	RETV(!m_articulation, nullptr);

	PxArticulationLink* link = m_articulation->createLink(parent
		, PxTransform(*(PxVec3*)&tfm.pos, *(PxQuat*)&tfm.rot));

	PxRigidActorExt::createExclusiveShape(*link
		, PxBoxGeometry(tfm.scale.x, tfm.scale.y, tfm.scale.z)
		, *m_physics.m_material);

	PxRigidBodyExt::updateMassAndInertia(*link, mass);

	return link;
}


void GetLocalFrame(const Transform& worldTm0, const Transform& worldTm1
	, const Vector3& jointPos, const Vector3& revoluteAxis
	, OUT physx::PxTransform& out0, OUT physx::PxTransform& out1)
{
	using namespace physx;

	Transform tfm0 = worldTm0;
	Transform tfm1 = worldTm1;

	Vector3 p0 = jointPos - tfm0.pos;
	Vector3 p1 = jointPos - tfm1.pos;
	Quaternion q0 = tfm0.rot.Inverse();
	Quaternion q1 = tfm1.rot.Inverse();

	if (revoluteAxis != Vector3::Zeroes)
	{
		Quaternion rot(revoluteAxis, Vector3(1, 0, 0));
		tfm0.rot *= rot;
		tfm1.rot *= rot;

		p0 = (tfm0.pos - jointPos) * rot + jointPos;
		p1 = (tfm1.pos - jointPos) * rot + jointPos;
		p0 = jointPos - p0;
		p1 = jointPos - p1;
		q0 = tfm0.rot.Inverse();
		q1 = tfm1.rot.Inverse();
	}

	const PxTransform localFrame0 = PxTransform(*(PxQuat*)&q0) * PxTransform(*(PxVec3*)&p0);
	const PxTransform localFrame1 = PxTransform(*(PxQuat*)&q1) * PxTransform(*(PxVec3*)&p1);

	out0 = localFrame0;
	out1 = localFrame1;
}


// create articulation joint
physx::PxArticulationJointReducedCoordinate* c3DView::CreateJoint(
	physx::PxArticulationLink* link
	, const Transform& worldTfm0, const Vector3& pivot0
	, const Transform& worldTfm1, const Vector3& pivot1
	, const physx::PxArticulationJointType::Enum jointType
)
{
	using namespace physx;

	RETV(!m_articulation || !link, nullptr);

	PxTransform localFrame0, localFrame1;
	const Vector3 jointPos = (pivot0 + pivot1) / 2.f;
	GetLocalFrame(worldTfm0, worldTfm1, jointPos, Vector3::Zeroes, localFrame0, localFrame1);

	PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
	joint->setJointType(jointType);
	joint->setParentPose(localFrame0);
	joint->setChildPose(localFrame1);

	return joint;
}


// initialize scissor lift2
void c3DView::InitScissorLift2()
{
	using namespace physx;

	m_jointPoss.clear();

	const PxReal runnerLength = 2.f;
	const PxReal placementDistance = 1.8f;

	const PxReal cosAng = (placementDistance) / (runnerLength);
	const PxReal angle = PxAcos(cosAng);
	const PxReal sinAng = PxSin(angle);

	const Quaternion leftRot(Vector3(1, 0, 0), -angle);
	const Quaternion rightRot(Vector3(1, 0, 0), angle);

	m_articulation = m_physics.m_physics->createArticulationReducedCoordinate();
	m_articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);

	//(1) Create base...
	const Transform baseTfm(Vector3(0.f, 0.25f, 0.f), Vector3(0.5f, 0.25f, 1.5f));
	PxArticulationLink* base = CreateLink(nullptr, baseTfm, 3.f);

	m_articulation->setSolverIterationCounts(32);

	const Transform leftRootTfm(Vector3(0.f, 0.55f, -0.9f), Vector3(0.5f, 0.05f, 0.05f));
	PxArticulationLink* leftRoot = CreateLink(base, leftRootTfm);
	const Transform rightRootTfm(Vector3(0.f, 0.55f, 0.9f), Vector3(0.5f, 0.05f, 0.05f));
	PxArticulationLink* rightRoot = CreateLink(base, rightRootTfm);

	PxArticulationJointReducedCoordinate* joint = 
		CreateJoint(leftRoot, baseTfm, baseTfm.pos
			, leftRootTfm, leftRootTfm.pos
			, PxArticulationJointType::eFIX);

	m_driveJoint = CreateJoint(rightRoot, baseTfm, baseTfm.pos
			, rightRootTfm, rightRootTfm.pos
			, PxArticulationJointType::ePRISMATIC);

	m_driveJoint->setMotion(PxArticulationAxis::eZ, PxArticulationMotion::eLIMITED);
	m_driveJoint->setLimitParams(PxArticulationAxis::eZ, PxArticulationLimit(-1.4f, 0.2f));
	m_driveJoint->setDriveParams(PxArticulationAxis::eZ, PxArticulationDrive(100000.f, 0.f, PX_MAX_F32));

	const PxU32 linkHeight = 3;
	PxArticulationLink* currLeft = leftRoot, * currRight = rightRoot;

	Transform leftParentTfm = leftRootTfm;
	Transform rightParentTfm = rightRootTfm;
	Vector3 leftParentAnchorPos = leftRootTfm.pos;
	Vector3 rightParentAnchorPos = rightRootTfm.pos;

	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		const Vector3 pos(0.5f, 0.55f + 0.1f * (1 + i) + sinAng + sinAng * (i * 2), 0.f);

		const Transform leftTfm(pos, Vector3(0.05f, 0.05f, 1.f), leftRot);
		PxArticulationLink* leftLink = CreateLink(currLeft, leftTfm);

		const Vector3 leftAnchorPos = leftParentAnchorPos + Vector3(0.5f, 0, 0);

		joint = CreateJoint(leftLink, leftParentTfm, leftParentAnchorPos
			, leftTfm, leftAnchorPos, PxArticulationJointType::eREVOLUTE);
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-PxPi, angle));


		const Transform rightTfm(pos, Vector3(0.05f, 0.05f, 1.f), rightRot);
		PxArticulationLink* rightLink = CreateLink(currRight, rightTfm);

		const Vector3 rightAnchorPos = rightParentAnchorPos + Vector3(0.5f, 0, 0);

		joint = CreateJoint(rightLink, rightParentTfm, rightParentAnchorPos
			, rightTfm, rightAnchorPos, PxArticulationJointType::eREVOLUTE);
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-angle, PxPi));


		m_jointPoss.push_back(leftAnchorPos);
		m_jointPoss.push_back(rightAnchorPos);

		leftParentTfm = rightTfm;
		leftParentAnchorPos = leftAnchorPos + Vector3(-0.5f, sinAng * 2 + 0.1f, 0);

		rightParentTfm = leftTfm;
		rightParentAnchorPos = rightAnchorPos + Vector3(-0.5f, sinAng * 2 + 0.1f, 0);


		const Vector3 pivot0 = leftTfm.pos;
		const Vector3 pivot1 = rightTfm.pos;
		const Vector3 revoluteAxis(1, 0, 0);

		PxTransform localFrame0, localFrame1;
		const Vector3 jointPos = (pivot0 + pivot1) / 2.f;
		GetLocalFrame(leftTfm, rightTfm, jointPos
			, revoluteAxis, localFrame0, localFrame1);

		PxD6Joint* d6Joint = PxD6JointCreate(*m_physics.m_physics
			, leftLink, localFrame0, rightLink, localFrame1);

		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currLeft = rightLink;
		currRight = leftLink;
	}//~for

	const Transform leftTfm(leftParentAnchorPos, Vector3(0.5f, 0.05f, 0.05f));
	PxArticulationLink* leftTop = CreateLink(currLeft, leftTfm);

	const Vector3 leftAnchorPos = leftParentAnchorPos + Vector3(0.5f, 0, 0);

	joint = CreateJoint(leftTop, leftParentTfm, leftParentAnchorPos
		, leftTfm, leftAnchorPos, PxArticulationJointType::eREVOLUTE);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);


	const Transform rightTfm(rightParentAnchorPos, Vector3(0.5f, 0.05f, 0.05f));
	PxArticulationLink* rightTop = CreateLink(currRight, rightTfm);

	const Vector3 rightAnchorPos = rightParentAnchorPos + Vector3(0.5f, 0, 0);

	joint = CreateJoint(rightTop, rightParentTfm, rightParentAnchorPos
		, rightTfm, rightAnchorPos, PxArticulationJointType::eREVOLUTE);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);


	currLeft = leftRoot;
	currRight = rightRoot;

	leftParentTfm = leftRootTfm;
	rightParentTfm = rightRootTfm;
	leftParentAnchorPos = leftRootTfm.pos;
	rightParentAnchorPos = rightRootTfm.pos;

	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		const Vector3 pos(-0.5f, 0.55f + 0.1f * (1 + i) + sinAng + sinAng * (i * 2), 0.f);

		const Transform leftTfm(pos, Vector3(0.05f, 0.05f, 1.f), leftRot);
		PxArticulationLink* leftLink = CreateLink(currLeft, leftTfm);

		const Vector3 leftAnchorPos = leftParentAnchorPos + Vector3(-0.5f, 0, 0);

		joint = CreateJoint(leftLink, leftParentTfm, leftParentAnchorPos
			, leftTfm, leftAnchorPos, PxArticulationJointType::eREVOLUTE);
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-PxPi, angle));


		const Transform rightTfm(pos, Vector3(0.05f, 0.05f, 1.f), rightRot);
		PxArticulationLink* rightLink = CreateLink(currRight, rightTfm);

		const Vector3 rightAnchorPos = rightParentAnchorPos + Vector3(-0.5f, 0, 0);

		joint = CreateJoint(rightLink, rightParentTfm, rightParentAnchorPos
			, rightTfm, rightAnchorPos, PxArticulationJointType::eREVOLUTE);
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-angle, PxPi));


		m_jointPoss.push_back(leftAnchorPos);
		m_jointPoss.push_back(rightAnchorPos);

		leftParentTfm = rightTfm;
		leftParentAnchorPos = leftAnchorPos + Vector3(0.5f, sinAng * 2 + 0.1f, 0);

		rightParentTfm = leftTfm;
		rightParentAnchorPos = rightAnchorPos + Vector3(0.5f, sinAng * 2 + 0.1f, 0);


		const Vector3 pivot0 = leftTfm.pos;
		const Vector3 pivot1 = rightTfm.pos;
		const Vector3 revoluteAxis(1, 0, 0);

		PxTransform localFrame0, localFrame1;
		const Vector3 jointPos = (pivot0 + pivot1) / 2.f;
		GetLocalFrame(leftTfm, rightTfm, jointPos
			, revoluteAxis, localFrame0, localFrame1);

		PxD6Joint* d6Joint = PxD6JointCreate(*m_physics.m_physics
			, leftLink, localFrame0, rightLink, localFrame1);

		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currLeft = rightLink;
		currRight = leftLink;
	}//~for

	{
		const Vector3 pivot0 = leftParentAnchorPos;
		const Vector3 pivot1 = leftParentAnchorPos + Vector3(0.5f, 0, 0);
		const Vector3 revoluteAxis(1, 0, 0);

		PxTransform localFrame0, localFrame1;
		const Vector3 jointPos = (pivot0 + pivot1) / 2.f;
		GetLocalFrame(leftParentTfm, leftTfm, jointPos
			, revoluteAxis, localFrame0, localFrame1);

		PxD6Joint* d6Joint = PxD6JointCreate(*m_physics.m_physics
			, currLeft, localFrame0, leftTop, localFrame1);

		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	}

	{
		const Vector3 pivot0 = rightParentAnchorPos;
		const Vector3 pivot1 = rightParentAnchorPos + Vector3(0.5f, 0, 0);
		const Vector3 revoluteAxis(1, 0, 0);

		PxTransform localFrame0, localFrame1;
		const Vector3 jointPos = (pivot0 + pivot1) / 2.f;
		GetLocalFrame(rightParentTfm, rightTfm, jointPos
			, revoluteAxis, localFrame0, localFrame1);

		PxD6Joint* d6Joint = PxD6JointCreate(*m_physics.m_physics
			, currRight, localFrame0, rightTop, localFrame1);

		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	}

	const Transform topPose(Vector3(0, leftTfm.pos.y + 0.15f, 0), Vector3(0.5f, 0.1f, 1.5f));
	PxArticulationLink* top = CreateLink(leftTop, topPose);

	joint = CreateJoint(top, leftTfm, leftTfm.pos
		, topPose, topPose.pos, PxArticulationJointType::eFIX);

	for (PxU32 i = 0; i < m_articulation->getNbLinks(); ++i)
	{
		PxArticulationLink* link;
		m_articulation->getLinks(&link, 1, i);

		link->setLinearDamping(0.2f);
		link->setAngularDamping(0.2f);

		link->setMaxAngularVelocity(20.f);
		link->setMaxLinearVelocity(100.f);

		if (link != top)
		{
			for (PxU32 b = 0; b < link->getNbShapes(); ++b)
			{
				PxShape* shape;
				link->getShapes(&shape, 1, b);

				shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));
			}
		}
	}

	// set up the box stack
	graphic::cRenderer& renderer = GetRenderer();

	const Vector3 halfExt(Vector3::Ones * 0.25f);
	const float density = 0.5f;
	const float contactOffset = 0.2f;

	const int id0 = m_physSync->SpawnBox(renderer, Transform(Vector3(-0.25f, 5.f, 0.5f), halfExt), density);
	const int id1 = m_physSync->SpawnBox(renderer, Transform(Vector3(0.25f, 5.f, 0.5f), halfExt), density);
	const int id2 = m_physSync->SpawnBox(renderer, Transform(Vector3(-0.25f, 4.5f, 0.5f), halfExt), density);
	const int id3 = m_physSync->SpawnBox(renderer, Transform(Vector3(0.25f, 4.5f, 0.5f), halfExt), density);
	const int id4 = m_physSync->SpawnBox(renderer, Transform(Vector3(-0.25f, 5.f, 0.f), halfExt), density);
	const int id5 = m_physSync->SpawnBox(renderer, Transform(Vector3(0.25f, 5.f, 0.f), halfExt), density);
	const int id6 = m_physSync->SpawnBox(renderer, Transform(Vector3(-0.25f, 4.5f, 0.f), halfExt), density);
	const int id7 = m_physSync->SpawnBox(renderer, Transform(Vector3(0.25f, 4.5f, 0.f), halfExt), density);

	if (phys::sSyncInfo *p = m_physSync->FindSyncInfo(id0))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id1))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id2))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id3))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id4))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id5))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id6))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id7))
		p->actor->SetContactOffset(contactOffset);


	m_physics.m_scene->addArticulation(*m_articulation);
}


// initialize scissor lift3
void c3DView::InitScissorLift3()
{
	using namespace physx;

	const PxReal runnerLength = 2.f;
	const PxReal placementDistance = 1.8f;

	const PxReal cosAng = (placementDistance) / (runnerLength);
	const PxReal angle = PxAcos(cosAng);
	const PxReal sinAng = PxSin(angle);

	const Quaternion leftRot(Vector3(1, 0, 0), -angle);
	const Quaternion rightRot(Vector3(1, 0, 0), angle);

	m_art.Create(m_physics, true, 32);	

	//(1) Create base...
	const Transform baseTfm(Vector3(0.f, 0.25f, 0.f), Vector3(0.5f, 0.25f, 1.5f));
	const int baseId = m_art.AddBoxLink(m_physics, -1, baseTfm, 3.f);

	const Transform leftRootTfm(Vector3(0.f, 0.55f, -0.9f), Vector3(0.5f, 0.05f, 0.05f));
	const int leftRootId = m_art.AddBoxLink(m_physics, baseId, leftRootTfm);
	const Transform rightRootTfm(Vector3(0.f, 0.55f, 0.9f), Vector3(0.5f, 0.05f, 0.05f));
	const int rightRootId = m_art.AddBoxLink(m_physics, baseId, rightRootTfm);

	m_art.AddJoint(leftRootId, phys::eJointType::Fixed, baseTfm.pos, leftRootTfm.pos);
	m_driveJoint = m_art.AddJoint(rightRootId, phys::eJointType::Prismatic, baseTfm.pos, rightRootTfm.pos);

	m_driveJoint->setMotion(PxArticulationAxis::eZ, PxArticulationMotion::eLIMITED);
	m_driveJoint->setLimitParams(PxArticulationAxis::eZ, PxArticulationLimit(-1.4f, 0.2f));
	m_driveJoint->setDriveParams(PxArticulationAxis::eZ, PxArticulationDrive(100000.f, 0.f, PX_MAX_F32));

	const PxU32 linkHeight = 3;
	int currLeft = leftRootId, currRight = rightRootId;
	PxArticulationJointReducedCoordinate* joint = nullptr;

	Transform leftParentTfm = leftRootTfm;
	Transform rightParentTfm = rightRootTfm;
	Vector3 leftParentAnchorPos = leftRootTfm.pos;
	Vector3 rightParentAnchorPos = rightRootTfm.pos;

	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		const Vector3 pos(0.5f, 0.55f + 0.1f * (1 + i) + sinAng + sinAng * (i * 2), 0.f);

		const Transform leftTfm(pos, Vector3(0.05f, 0.05f, 1.f), leftRot);
		const int leftLinkId = m_art.AddBoxLink(m_physics, currLeft, leftTfm);

		const Vector3 leftAnchorPos = leftParentAnchorPos + Vector3(0.5f, 0, 0);
		joint = m_art.AddJoint(leftLinkId, phys::eJointType::Revolute, leftParentAnchorPos, leftAnchorPos);

		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-PxPi, angle));

		const Transform rightTfm(pos, Vector3(0.05f, 0.05f, 1.f), rightRot);
		const int rightLinkId = m_art.AddBoxLink(m_physics, currRight, rightTfm);

		const Vector3 rightAnchorPos = rightParentAnchorPos + Vector3(0.5f, 0, 0);
		joint = m_art.AddJoint(rightLinkId, phys::eJointType::Revolute, rightParentAnchorPos, rightAnchorPos);

		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-angle, PxPi));

		leftParentTfm = rightTfm;
		leftParentAnchorPos = leftAnchorPos + Vector3(-0.5f, sinAng * 2 + 0.1f, 0);

		rightParentTfm = leftTfm;
		rightParentAnchorPos = rightAnchorPos + Vector3(-0.5f, sinAng * 2 + 0.1f, 0);

		physx::PxD6Joint *d6Joint = 
			m_art.AddD6Joint(m_physics, leftLinkId, leftTfm.pos, rightLinkId, rightTfm.pos);

		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currLeft = rightLinkId;
		currRight = leftLinkId;
	}//~for

	const Transform leftTfm(leftParentAnchorPos, Vector3(0.5f, 0.05f, 0.05f));
	const int leftTop = m_art.AddBoxLink(m_physics, currLeft, leftTfm);

	const Vector3 leftAnchorPos = leftParentAnchorPos + Vector3(0.5f, 0, 0);

	joint = m_art.AddJoint(leftTop, phys::eJointType::Revolute
		, leftParentAnchorPos, leftAnchorPos);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);


	const Transform rightTfm(rightParentAnchorPos, Vector3(0.5f, 0.05f, 0.05f));
	const int rightTop = m_art.AddBoxLink(m_physics, currRight, rightTfm);

	const Vector3 rightAnchorPos = rightParentAnchorPos + Vector3(0.5f, 0, 0);

	joint = m_art.AddJoint(rightTop, phys::eJointType::Revolute
		, rightParentAnchorPos, rightAnchorPos);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);


	currLeft = leftRootId;
	currRight = rightRootId;

	leftParentTfm = leftRootTfm;
	rightParentTfm = rightRootTfm;
	leftParentAnchorPos = leftRootTfm.pos;
	rightParentAnchorPos = rightRootTfm.pos;

	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		const Vector3 pos(-0.5f, 0.55f + 0.1f * (1 + i) + sinAng + sinAng * (i * 2), 0.f);

		const Transform leftTfm(pos, Vector3(0.05f, 0.05f, 1.f), leftRot);
		const int leftLinkId = m_art.AddBoxLink(m_physics, currLeft, leftTfm);

		const Vector3 leftAnchorPos = leftParentAnchorPos + Vector3(-0.5f, 0, 0);
		joint = m_art.AddJoint(leftLinkId, phys::eJointType::Revolute, leftParentAnchorPos, leftAnchorPos);

		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-PxPi, angle));

		const Transform rightTfm(pos, Vector3(0.05f, 0.05f, 1.f), rightRot);
		const int rightLinkId = m_art.AddBoxLink(m_physics, currRight, rightTfm);

		const Vector3 rightAnchorPos = rightParentAnchorPos + Vector3(-0.5f, 0, 0);
		joint = m_art.AddJoint(rightLinkId, phys::eJointType::Revolute, rightParentAnchorPos, rightAnchorPos);

		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimitParams(PxArticulationAxis::eTWIST, PxArticulationLimit(-angle, PxPi));

		leftParentTfm = rightTfm;
		leftParentAnchorPos = leftAnchorPos + Vector3(0.5f, sinAng * 2 + 0.1f, 0);

		rightParentTfm = leftTfm;
		rightParentAnchorPos = rightAnchorPos + Vector3(0.5f, sinAng * 2 + 0.1f, 0);

		physx::PxD6Joint* d6Joint =
			m_art.AddD6Joint(m_physics, leftLinkId, leftTfm.pos, rightLinkId, rightTfm.pos);

		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currLeft = rightLinkId;
		currRight = leftLinkId;
	}//~for

	{
		physx::PxD6Joint* d6Joint =
			m_art.AddD6Joint(m_physics, currLeft, leftParentAnchorPos
				, leftTop, leftParentAnchorPos + Vector3(0.5f, 0, 0));

		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	}

	{
		physx::PxD6Joint* d6Joint =
			m_art.AddD6Joint(m_physics, currRight, rightParentAnchorPos
				, rightTop, rightParentAnchorPos + Vector3(0.5f, 0, 0));

		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	}

	const Transform topPose(Vector3(0, leftTfm.pos.y + 0.15f, 0), Vector3(0.5f, 0.1f, 1.5f));
	const int top = m_art.AddBoxLink(m_physics, leftTop, topPose);

	joint = m_art.AddJoint(top, phys::eJointType::Fixed, leftTfm.pos, topPose.pos);
	m_art.SetAttribute(0.2f, 0.2f, 100.f, 20.f);

	for (auto &kv : m_art.m_links)
	{
		if (kv.first != top)
		{
			PxArticulationLink* link = kv.second.link;

			for (PxU32 b = 0; b < link->getNbShapes(); ++b)
			{
				PxShape* shape;
				link->getShapes(&shape, 1, b);

				shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));
			}
		}
	}

	// set up the box stack
	graphic::cRenderer& renderer = GetRenderer();

	const Vector3 halfExt(Vector3::Ones * 0.25f);
	const float density = 0.5f;
	const float contactOffset = 0.2f;

	const int id0 = m_physSync->SpawnBox(renderer, Transform(Vector3(-0.25f, 5.f, 0.5f), halfExt), density);
	const int id1 = m_physSync->SpawnBox(renderer, Transform(Vector3(0.25f, 5.f, 0.5f), halfExt), density);
	const int id2 = m_physSync->SpawnBox(renderer, Transform(Vector3(-0.25f, 4.5f, 0.5f), halfExt), density);
	const int id3 = m_physSync->SpawnBox(renderer, Transform(Vector3(0.25f, 4.5f, 0.5f), halfExt), density);
	const int id4 = m_physSync->SpawnBox(renderer, Transform(Vector3(-0.25f, 5.f, 0.f), halfExt), density);
	const int id5 = m_physSync->SpawnBox(renderer, Transform(Vector3(0.25f, 5.f, 0.f), halfExt), density);
	const int id6 = m_physSync->SpawnBox(renderer, Transform(Vector3(-0.25f, 4.5f, 0.f), halfExt), density);
	const int id7 = m_physSync->SpawnBox(renderer, Transform(Vector3(0.25f, 4.5f, 0.f), halfExt), density);

	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id0))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id1))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id2))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id3))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id4))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id5))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id6))
		p->actor->SetContactOffset(contactOffset);
	if (phys::sSyncInfo* p = m_physSync->FindSyncInfo(id7))
		p->actor->SetContactOffset(contactOffset);

	m_art.AddScene(m_physics);
}


static bool gClosing = true;

void c3DView::OnUpdate(const float deltaSeconds)
{
	if (m_isSimulation)
	{
		if (1 && m_driveJoint)
		{
			using namespace physx;
			const PxReal dt = 1.0f / 60.f;
			PxReal driveValue = m_driveJoint->getDriveTarget(PxArticulationAxis::eZ);

			if (gClosing && driveValue < -1.2f)
				gClosing = false;
			else if (!gClosing && driveValue > 0.f)
				gClosing = true;

			if (gClosing)
				driveValue -= dt * 0.25f;
			else
				driveValue += dt * 0.25f;
			m_driveJoint->setDriveTarget(PxArticulationAxis::eZ, driveValue);
		}

		m_physics.PreUpdate(deltaSeconds);
		m_physics.PostUpdate(deltaSeconds);
	}
}


void c3DView::OnPreRender(const float deltaSeconds)
{
	cRenderer &renderer = GetRenderer();
	cAutoCam cam(&m_camera);

	renderer.UnbindShaderAll();
	renderer.UnbindTextureAll();

	GetMainCamera().Bind(renderer);
	GetMainLight().Bind(renderer);	

	if (m_renderTarget.Begin(renderer))
	{
		CommonStates states(renderer.GetDevice());
		renderer.GetDevContext()->RSSetState(states.CullNone());

		if (m_physSync)
		{
			for (auto &p : m_physSync->m_syncs)
				p->node->Render(renderer);
		}

		using namespace physx;

		m_box.SetColor(cColor::WHITE);

		PxShape* shapes[128];
		uint nbArticulations = m_physics.m_scene->getNbArticulations();
		for (uint i = 0; i < nbArticulations; i++)
		{
			PxArticulationReducedCoordinate* articulation;
			m_physics.m_scene->getArticulations(&articulation, 1, i);

			const uint nbLinks = articulation->getNbLinks();
			PxArray<PxArticulationLink*> links(nbLinks);
			articulation->getLinks(&links[0], nbLinks);

			for (auto& actor : links)
			{
				const uint nbShapes = actor->getNbShapes();
				if (nbShapes > 128) continue;
				actor->getShapes(shapes, nbShapes);

				for (uint k = 0; k < nbShapes; ++k)
				{
					const PxGeometry& geom = shapes[k]->getGeometry();
					switch (geom.getType())
					{
					case PxGeometryType::eBOX:
					{
						const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);
						const Vector3 scale(boxGeom.halfExtents.x, boxGeom.halfExtents.y, boxGeom.halfExtents.z);

						const PxTransform pose = actor->getGlobalPose();
						const Transform tm = Transform(Vector3(), scale) *
							Transform(*(Quaternion*)&pose.q) * Transform(*(Vector3*)&pose.p);
						m_box.SetCube(tm);
						m_box.Render(renderer);
					}
					break;
					default:
						break;
					}

				}//~for shapes
			}//~for links
		}//~for articulations


		m_box.SetColor(cColor::GREEN);
		for (auto& pos : m_jointPoss)
		{
			Transform tm(pos, Vector3::Ones * 0.01f);
			m_box.SetCube(tm);
			m_box.Render(renderer);
		}


		if (m_showGrid)
			m_grid.Render(renderer);
		renderer.RenderAxis2();
		renderer.GetDevContext()->RSSetState(states.CullCounterClockwise());
	}
	m_renderTarget.End(renderer);
}


void c3DView::OnRender(const float deltaSeconds)
{
	ImVec2 pos = ImGui::GetCursorScreenPos();
	m_viewPos = { (int)(pos.x), (int)(pos.y) };
	m_viewRect = { pos.x + 5, pos.y, pos.x + m_rect.Width() - 30, pos.y + m_rect.Height() - 42 };

	// HUD
	bool isOpen = true;
	ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration
		| ImGuiWindowFlags_NoBackground
		;

	ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
	ImGui::Image(m_renderTarget.m_resolvedSRV, ImVec2(m_rect.Width() - 15, m_rect.Height() - 42));

	// Render Information
	ImGui::SetNextWindowPos(ImVec2(pos.x, pos.y));
	ImGui::SetNextWindowBgAlpha(0.f);
	ImGui::SetNextWindowSize(ImVec2(min(m_viewRect.Width(), 350.f), m_viewRect.Height()));
	if (ImGui::Begin("Map Information", &isOpen, flags))
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Checkbox("grid", &m_showGrid);
		ImGui::Checkbox("simulation", &m_isSimulation);		
		ImGui::Text("Press SpaceBar");
		ImGui::End();
	}

	ImGui::PopStyleColor();
}


void c3DView::OnResizeEnd(const framework::eDockResize::Enum type, const sRectf &rect)
{
	if (type == eDockResize::DOCK_WINDOW)
	{
		m_owner->RequestResetDeviceNextFrame();
	}
}


void c3DView::UpdateLookAt()
{
	GetMainCamera().MoveCancel();

	const float centerX = GetMainCamera().m_width / 2;
	const float centerY = GetMainCamera().m_height / 2;
	const Ray ray = GetMainCamera().GetRay((int)centerX, (int)centerY);
	const Plane groundPlane(Vector3(0, 1, 0), 0);
	const float distance = groundPlane.Collision(ray.dir);
	if (distance < -0.2f)
	{
		GetMainCamera().m_lookAt = groundPlane.Pick(ray.orig, ray.dir);
	}
	else
	{ // horizontal viewing
		const Vector3 lookAt = GetMainCamera().m_eyePos + GetMainCamera().GetDirection() * 50.f;
		GetMainCamera().m_lookAt = lookAt;
	}

	GetMainCamera().UpdateViewMatrix();
}


// 휠을 움직였을 때,
// 카메라 앞에 박스가 있다면, 박스 정면에서 멈춘다.
void c3DView::OnWheelMove(const float delta, const POINT mousePt)
{
	UpdateLookAt();

	float len = 0;
	const Plane groundPlane(Vector3(0, 1, 0), 0);
	const Ray ray = GetMainCamera().GetRay(mousePt.x, mousePt.y);
	Vector3 lookAt = groundPlane.Pick(ray.orig, ray.dir);
	len = (ray.orig - lookAt).Length();

	const int lv = 10;
	const float zoomLen = min(len * 0.1f, (float)(2 << (16 - lv)));

	GetMainCamera().Zoom(ray.dir, (delta < 0) ? -zoomLen : zoomLen);
}


// Handling Mouse Move Event
void c3DView::OnMouseMove(const POINT mousePt)
{
	const POINT delta = { mousePt.x - m_mousePos.x, mousePt.y - m_mousePos.y };
	
	const POINT prevMousePt = m_mousePos;
	m_mousePos = mousePt;
	if (ImGui::IsMouseHoveringRect(ImVec2(-1000, -1000), ImVec2(1000, 200), false))
		return;

	if (m_mouseDown[0])
	{
		const Plane ground(Vector3(0, 1, 0), 0);
		const Ray ray0 = GetMainCamera().GetRay(prevMousePt.x, prevMousePt.y);
		const Ray ray1 = GetMainCamera().GetRay(mousePt.x, mousePt.y);
		const Vector3 p0 = ground.Pick(ray0.orig, ray0.dir);
		const Vector3 p1 = ground.Pick(ray1.orig, ray1.dir);

		Vector3 dir = GetMainCamera().GetDirection();
		dir.y = 0;
		dir.Normalize();

		Vector3 right = GetMainCamera().GetRight();
		right.y = 0;
		right.Normalize();

		const float df = dir.DotProduct((p1 - p0));
		const float dr = right.DotProduct((p1 - p0));
		GetMainCamera().MoveRight(-dr);
		GetMainCamera().MoveFrontHorizontal(-df);

		//Vector3 dir = GetMainCamera().GetDirection();
		//Vector3 right = GetMainCamera().GetRight();
		//dir.y = 0;
		//dir.Normalize();
		//right.y = 0;
		//right.Normalize();

		//GetMainCamera().MoveRight(-delta.x * m_rotateLen * 0.001f);
		//GetMainCamera().MoveFrontHorizontal(delta.y * m_rotateLen * 0.001f);
	}
	else if (m_mouseDown[1])
	{
		const float scale = 0.005f;
		m_camera.Yaw2(delta.x * scale, Vector3(0, 1, 0));
		m_camera.Pitch2(delta.y * scale, Vector3(0, 1, 0));
	}
	else if (m_mouseDown[2])
	{
		const float len = GetMainCamera().GetDistance();
		GetMainCamera().MoveRight(-delta.x * len * 0.001f);
		GetMainCamera().MoveUp(delta.y * len * 0.001f);
	}
}


// Handling Mouse Button Down Event
void c3DView::OnMouseDown(const sf::Mouse::Button &button, const POINT mousePt)
{
	m_mousePos = mousePt;
	UpdateLookAt();
	SetCapture();

	const Ray ray = GetMainCamera().GetRay(mousePt.x, mousePt.y);
	const Plane groundPlane(Vector3(0, 1, 0), 0);
	const Vector3 target = groundPlane.Pick(ray.orig, ray.dir);
	m_rotateLen = (target - ray.orig).Length();

	switch (button)
	{
	case sf::Mouse::Left:
	{
		m_mouseDown[0] = true;
	}
	break;

	case sf::Mouse::Right:
	{
		m_mouseDown[1] = true;

		const Ray ray = GetMainCamera().GetRay(mousePt.x, mousePt.y);
		Vector3 target = groundPlane.Pick(ray.orig, ray.dir);
		const float len = (GetMainCamera().GetEyePos() - target).Length();
	}
	break;

	case sf::Mouse::Middle:
		m_mouseDown[2] = true;
		break;
	}
}


void c3DView::OnMouseUp(const sf::Mouse::Button &button, const POINT mousePt)
{
	const POINT delta = { mousePt.x - m_mousePos.x, mousePt.y - m_mousePos.y };
	m_mousePos = mousePt;
	ReleaseCapture();

	switch (button)
	{
	case sf::Mouse::Left:
		m_mouseDown[0] = false;
		break;
	case sf::Mouse::Right:
		m_mouseDown[1] = false;
		break;
	case sf::Mouse::Middle:
		m_mouseDown[2] = false;
		break;
	}
}


void c3DView::OnEventProc(const sf::Event &evt)
{
	ImGuiIO& io = ImGui::GetIO();
	switch (evt.type)
	{
	case sf::Event::KeyPressed:
		switch (evt.key.cmd)
		{
		case sf::Keyboard::Return:
			break;
		case sf::Keyboard::Space:
		{
			using namespace physx;
			phys::sSyncInfo *box = m_physSync->FindSyncInfo(m_boxId);
	
			//PxVec3 force(10, 0, 0);
			//box->actor->m_dynamic->addForce(force, PxForceMode::eIMPULSE, true);
			//PxRigidBodyExt::addForceAtPos(*box->actor->m_dynamic
			//	, PxVec3(-1.f, 0, 0), PxVec3(0.5f,0,0) );

			PxVec3 force(0, 100, 0);
			if (physx::PxRigidDynamic *p = box->actor->m_actor->is<physx::PxRigidDynamic>())
				p->addTorque(force, PxForceMode::eIMPULSE, true);
		}
		break;
		}
		break;

	case sf::Event::MouseMoved:
	{
		cAutoCam cam(&m_camera);

		POINT curPos;
		GetCursorPos(&curPos); // sf::event mouse position has noise so we use GetCursorPos() function
		ScreenToClient(m_owner->getSystemHandle(), &curPos);
		POINT pos = { curPos.x - m_viewPos.x, curPos.y - m_viewPos.y };
		OnMouseMove(pos);
	}
	break;

	case sf::Event::MouseButtonPressed:
	case sf::Event::MouseButtonReleased:
	{
		cAutoCam cam(&m_camera);

		POINT curPos;
		GetCursorPos(&curPos); // sf::event mouse position has noise so we use GetCursorPos() function
		ScreenToClient(m_owner->getSystemHandle(), &curPos);
		const POINT pos = { curPos.x - m_viewPos.x, curPos.y - m_viewPos.y };
		const sRectf viewRect = GetWindowSizeAvailible(true);

		if (sf::Event::MouseButtonPressed == evt.type)
		{
			if (viewRect.IsIn((float)curPos.x, (float)curPos.y))
				OnMouseDown(evt.mouseButton.button, pos);
		}
		else
		{
			// 화면밖에 마우스가 있더라도 Capture 상태일 경우 Up 이벤트는 받게한다.
			if (viewRect.IsIn((float)curPos.x, (float)curPos.y)
				|| (this == GetCapture()))
				OnMouseUp(evt.mouseButton.button, pos);
		}
	}
	break;

	case sf::Event::MouseWheelScrolled:
	{
		cAutoCam cam(&m_camera);

		POINT curPos;
		GetCursorPos(&curPos); // sf::event mouse position has noise so we use GetCursorPos() function
		ScreenToClient(m_owner->getSystemHandle(), &curPos);
		const POINT pos = { curPos.x - m_viewPos.x, curPos.y - m_viewPos.y };
		OnWheelMove(evt.mouseWheelScroll.delta, pos);
	}
	break;

	case sf::Event::Gestured:
	{
		POINT curPos = { evt.gesture.x, evt.gesture.y };
		ScreenToClient(m_owner->getSystemHandle(), &curPos);
		const POINT pos = { curPos.x - m_viewPos.x, curPos.y - m_viewPos.y };
	}
	break;

	}
}


void c3DView::OnResetDevice()
{
	cRenderer &renderer = GetRenderer();

	// update viewport
	sRectf viewRect = { 0, 0, m_rect.Width() - 15, m_rect.Height() - 50 };
	m_camera.SetViewPort(viewRect.Width(), viewRect.Height());

	cViewport vp = GetRenderer().m_viewPort;
	vp.m_vp.Width = viewRect.Width();
	vp.m_vp.Height = viewRect.Height();
	m_renderTarget.Create(renderer, vp, DXGI_FORMAT_R8G8B8A8_UNORM, true, true, DXGI_FORMAT_D24_UNORM_S8_UINT);
}
