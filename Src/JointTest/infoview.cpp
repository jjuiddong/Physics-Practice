
#include "stdafx.h"
#include "infoview.h"


cInformationView::cInformationView(const StrId &name)
	: framework::cDockWindow(name)
	, m_distance(1.f)
	, m_radius(0.5f)
	, m_item(0)
	, m_isKinematic0(false)
	, m_isKinematic1(false)
	, m_isLimitJoint(true)
	, m_pos0(0, 10, 0)
	, m_pos1(0, 10, 0)
	, m_scale0(0.5f, 0.5f, 0.5f)
	, m_scale1(0.5f, 0.5f, 0.5f)
{
}

cInformationView::~cInformationView()
{
}


void cInformationView::OnUpdate(const float deltaSeconds)
{
}


void cInformationView::OnRender(const float deltaSeconds)
{
	graphic::cRenderer &renderer = GetRenderer();

	if (ImGui::Button("Spawn Object"))
	{
		const Vector3 pos(0, 10, 0);
		const float scale = 0.5f;
		static int idx = 2;
		graphic::cNode *node = nullptr;
		switch (idx)
		{
		case 0:
			node = g_physics.SpawnBox(renderer, pos, Vector3::Ones * scale);
			g_physics.m_actors.push_back({ g_physics.CreateBox(pos, Vector3::Ones*scale)
				, "cube", node });
			break;

		case 1:
			node = g_physics.SpawnSphere(renderer, pos, scale);
			g_physics.m_actors.push_back({ g_physics.CreateSphere(pos, scale), "sphere", node });
			break;

		case 2:
			node = g_physics.SpawnCapsule(renderer, pos, scale, scale*2.f);
			g_physics.m_actors.push_back({ g_physics.CreateCapsule(pos, scale, scale*2.f), "capsule", node });
			break;
		}
		++idx;
		idx %= 3;
	}

	ImGui::SameLine(200);
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.1f, 0.1f, 1.f));
	ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.3f, 0.1f, 0.1f, 1.f));
	if (ImGui::Button("Clear"))
	{
		g_physics.ClearPhysicsObject();
	}
	ImGui::PopStyleColor(3);

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	RenderActorEdit();
	ImGui::Separator();
	ImGui::Spacing();

	RenderFixedJoint();
	RenderSphericalJoint();
	RenderRevoluteJoint();
	RenderPrismaticJoint();
	RenderDistanceJoint();
	RenderD6Joint();
}


void cInformationView::RenderActorEdit()
{
	ImGui::Combo("Model", &m_item, "Box\0Sphere\0Capsule\0\0");
	const bool edit0 = ImGui::DragFloat("Distance", &m_distance, 0.01f, 0, 100.f);
	ImGui::DragFloat("Radius", &m_radius, 0.01f, 0, 100.f);
	ImGui::Text("Kinematic");
	ImGui::SameLine();
	ImGui::Checkbox("Actor0", &m_isKinematic0);
	ImGui::SameLine();
	ImGui::Checkbox("Actor1", &m_isKinematic1);
	ImGui::Checkbox("Joint Limit", &m_isLimitJoint);
	ImGui::Spacing();

	if (edit0)
	{
		m_pos0 = Vector3(m_distance, m_pos0.y, m_pos0.z);
		m_pos1 = Vector3(-m_distance, m_pos1.y, m_pos1.z);
	}

	ImGui::DragFloat3("Pos0", (float*)&m_pos0, 0.001f);
	ImGui::DragFloat3("Scale0 (Box)", (float*)&m_scale0, 0.001f);
	ImGui::DragFloat3("Pos1", (float*)&m_pos1, 0.001f);
	ImGui::DragFloat3("Scale1 (Box)", (float*)&m_scale1, 0.001f);
}


// create graphic, physics object
cInformationView::sSpawnObj cInformationView::SpawnObject()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	graphic::cNode *node0 = nullptr, *node1 = nullptr;
	PxRigidDynamic *actor0 = nullptr, *actor1 = nullptr;
	switch (m_item)
	{
	case 0:
		node0 = g_physics.SpawnBox(renderer, m_pos0, m_scale0);
		node1 = g_physics.SpawnBox(renderer, m_pos1, m_scale1);
		actor0 = g_physics.CreateBox(m_pos0, m_scale0);
		actor1 = g_physics.CreateBox(m_pos1, m_scale1);
		break;

	case 1:
		node0 = g_physics.SpawnSphere(renderer, m_pos0, m_radius);
		node1 = g_physics.SpawnSphere(renderer, m_pos1, m_radius);
		actor0 = g_physics.CreateSphere(m_pos0, m_radius);
		actor1 = g_physics.CreateSphere(m_pos1, m_radius);
		break;

	case 2:
		node0 = g_physics.SpawnCapsule(renderer, m_pos0, m_radius, m_radius*2.f);
		node1 = g_physics.SpawnCapsule(renderer, m_pos1, m_radius, m_radius*2.f);
		actor0 = g_physics.CreateCapsule(m_pos0, m_radius, m_radius*2.f);
		actor1 = g_physics.CreateCapsule(m_pos1, m_radius, m_radius*2.f);
		break;
	}

	sSpawnObj obj;
	obj.node0 = node0;
	obj.node1 = node1;
	obj.actor0 = actor0;
	obj.actor1 = actor1;

	if (m_pos0 == m_pos1)
	{
		m_pos0 = Vector3(m_distance, m_pos0.y, m_pos0.z);
		m_pos1 = Vector3(-m_distance, m_pos1.y, m_pos1.z);
	}

	return obj;
}


void cInformationView::RenderFixedJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	//ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Fixed Joint"))
	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.6f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.8f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.3f, 0.3f, 0.1f, 1.f));
		if (ImGui::Button("Create Fixed Joint"))
		{
			sSpawnObj result = SpawnObject();
			if (result.node0)
			{
				const Transform worldTfm0(m_pos0);
				const Transform worldTfm1(m_pos1);
				PxTransform localFrame0, localFrame1;
				GetLocalFrame(worldTfm0, worldTfm1, Vector3::Zeroes
					, localFrame0, localFrame1);

				PxFixedJoint *joint = PxFixedJointCreate(*g_physics.m_physics
					, result.actor0, localFrame0
					, result.actor1, localFrame1);

				result.actor0->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic0);
				result.actor1->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic1);

				g_physics.m_actors.push_back({ result.actor0, "cube", result.node0 });
				g_physics.m_actors.push_back({ result.actor1, "cube", result.node1 });
				g_physics.AddJoint(joint);
			}
		}
		ImGui::PopStyleColor(3);

		ImGui::Spacing();
		ImGui::Spacing();
	}
}


void cInformationView::RenderSphericalJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	//ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Spherical Joint"))
	{
		static Vector2 limit(PxPi / 2.f, PxPi / 6.f);
		ImGui::Text("Limit Cone (Radian)");
		ImGui::DragFloat("Y Limit Angle", &limit.x, 0.001f);
		ImGui::DragFloat("Z Limit Angle", &limit.y, 0.001f);

		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.6f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.8f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.3f, 0.3f, 0.1f, 1.f));
		if (ImGui::Button("Create Sperical Joint"))
		{
			sSpawnObj result = SpawnObject();
			if (result.node0)
			{
				const Transform worldTfm0(m_pos0);
				const Transform worldTfm1(m_pos1);
				PxTransform localFrame0, localFrame1;
				GetLocalFrame(worldTfm0, worldTfm1, Vector3::Zeroes
					, localFrame0, localFrame1);

				PxSphericalJoint *joint = PxSphericalJointCreate(*g_physics.m_physics
					, result.actor0, localFrame0
					, result.actor1, localFrame1);

				if (m_isLimitJoint)
				{
					joint->setLimitCone(PxJointLimitCone(limit.x, limit.y, 0.01f));
					joint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
				}

				result.actor0->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic0);
				result.actor1->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic1);

				g_physics.m_actors.push_back({ result.actor0, "cube", result.node0 });
				g_physics.m_actors.push_back({ result.actor1, "cube", result.node1 });
				g_physics.AddJoint(joint);
			}
		}
		ImGui::PopStyleColor(3);

		ImGui::Spacing();
		ImGui::Spacing();
	}
}


void cInformationView::RenderRevoluteJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	//ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Revolute Joint"))
	{
		static Vector2 limit(-PxPi / 4.f, PxPi / 4.f);
		static bool isDrive = false;
		static float velocity = 1.f;
		ImGui::Text("Angular Limit (Radian)");
		ImGui::DragFloat("Lower Limit Angle", &limit.x, 0.001f);
		ImGui::DragFloat("Upper Limit Angle", &limit.y, 0.001f);
		ImGui::Checkbox("Drive", &isDrive);
		ImGui::DragFloat("Velocity", &velocity, 0.001f);

		const char *axisStr = "X\0Y\0Z\0\0";
		const static Vector3 axis[3] = { Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1) };	
		static int axisIdx = 0;
		ImGui::Combo("Revolute Axis", &axisIdx, axisStr);

		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.6f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.8f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.3f, 0.3f, 0.1f, 1.f));
		if (ImGui::Button("Create Revolute Joint"))
		{
			sSpawnObj result = SpawnObject();
			if (result.node0)
			{
				const Vector3 revoluteAxis = axis[axisIdx];

				const Transform worldTfm0(m_pos0);
				const Transform worldTfm1(m_pos1);
				PxTransform localFrame0, localFrame1;
				GetLocalFrame(worldTfm0, worldTfm1, revoluteAxis
					, localFrame0, localFrame1);

				PxRevoluteJoint *joint = PxRevoluteJointCreate(*g_physics.m_physics
					, result.actor0, localFrame0
					, result.actor1, localFrame1);

				if (m_isLimitJoint)
				{
					joint->setLimit(PxJointAngularLimitPair(limit.x, limit.y, 0.01f));
					joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
				}

				if (isDrive)
				{
					joint->setDriveVelocity(velocity);
					joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, isDrive);
				}

				result.actor0->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic0);
				result.actor1->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic1);

				g_physics.m_actors.push_back({ result.actor0, "cube", result.node0 });
				g_physics.m_actors.push_back({ result.actor1, "cube", result.node1 });
				g_physics.AddJoint(joint);
			}
		}
		ImGui::PopStyleColor(3);

		ImGui::Spacing();
		ImGui::Spacing();
	}
}


void cInformationView::RenderPrismaticJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	//ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Prismatic Joint"))
	{
		static Vector2 limit(-PxPi / 4.f, PxPi / 4.f);
		static PxTolerancesScale tolerance;
		ImGui::Text("Linear Limit");
		ImGui::DragFloat("Lower Limit", &limit.x, 0.001f);
		ImGui::DragFloat("Upper Limit", &limit.y, 0.001f);
		ImGui::DragFloat("length Tolerance ", &tolerance.length, 0.001f);
		ImGui::DragFloat("mass Tolerance ", &tolerance.mass, 0.001f);
		ImGui::DragFloat("speed Tolerance ", &tolerance.speed, 0.001f);

		const char *axisStr = "X\0Y\0Z\0\0";
		const static Vector3 axis[3] = { Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1) };
		static int axisIdx = 0;
		ImGui::Combo("Prismatic Axis", &axisIdx, axisStr);

		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.6f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.8f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.3f, 0.3f, 0.1f, 1.f));
		if (ImGui::Button("Create"))
		{
			sSpawnObj result = SpawnObject();
			if (result.node0)
			{
				const Vector3 lienarAxis = axis[axisIdx];
				const Transform worldTfm0(m_pos0);
				const Transform worldTfm1(m_pos1);
				PxTransform localFrame0, localFrame1;
				GetLocalFrame(worldTfm0, worldTfm1, lienarAxis
					, localFrame0, localFrame1);

				PxPrismaticJoint *joint = PxPrismaticJointCreate(*g_physics.m_physics
					, result.actor0, localFrame0
					, result.actor1, localFrame1);

				if (m_isLimitJoint)
				{
					joint->setLimit(PxJointLinearLimitPair(tolerance, limit.x, limit.y, 0.01f));
					joint->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
				}

				result.actor0->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic0);
				result.actor1->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic1);

				g_physics.m_actors.push_back({ result.actor0, "cube", result.node0 });
				g_physics.m_actors.push_back({ result.actor1, "cube", result.node1 });
				g_physics.AddJoint(joint);
			}
		}
		ImGui::PopStyleColor(3);
	}
}


void cInformationView::RenderDistanceJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	//ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Distance Joint"))
	{
		static float maxDistance = 3.f;
		ImGui::DragFloat("Max Distance", &maxDistance, 0.01f);

		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.6f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.8f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.3f, 0.3f, 0.1f, 1.f));
		if (ImGui::Button("Create Distance Joint"))
		{
			sSpawnObj result = SpawnObject();
			if (result.node0)
			{
				const Transform worldTfm0(m_pos0);
				const Transform worldTfm1(m_pos1);
				PxTransform localFrame0, localFrame1;
				GetLocalFrame(worldTfm0, worldTfm1, Vector3::Zeroes
					, localFrame0, localFrame1);

				PxDistanceJoint *joint = PxDistanceJointCreate(*g_physics.m_physics
					, result.actor0, localFrame0
					, result.actor1, localFrame1);

				if (m_isLimitJoint)
				{
					joint->setMaxDistance(maxDistance);
					joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
				}

				result.actor0->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic0);
				result.actor1->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic1);

				g_physics.m_actors.push_back({ result.actor0, "cube", result.node0 });
				g_physics.m_actors.push_back({ result.actor1, "cube", result.node1 });
				g_physics.AddJoint(joint);
			}
		}
		ImGui::PopStyleColor(3);

		ImGui::Spacing();
		ImGui::Spacing();
	}
}


void cInformationView::RenderD6Joint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	//ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("D6 Joint"))
	{
		static int motionVal[6] = { 0, 0, 0, 0, 0, 0 };
		const char *motionStr = "Lock\0Limit\0Free\0\0";
		const char *labels[6] = { "X   ", "Y   ", "Z   ", "Twist", "Swing1", "Swing2" };
		const char *labels2[6] = { "dX   ", "dY   ", "dZ   ", "dTwist", "dSwing1", "dSwing2" };
		
		static PxJointLinearLimit linearLimit(1.f, PxSpring(1.f,1.f));
		static bool isLinearLimit = false;
		ImGui::Text("Linear Limit");
		ImGui::Checkbox("_Linear Limit", &isLinearLimit);
		ImGui::DragFloat("Extend", &linearLimit.value, 0.001f);
		ImGui::DragFloat("Stiffness", &linearLimit.stiffness, 0.001f);
		ImGui::DragFloat("Damping", &linearLimit.damping, 0.001f);

		static PxJointAngularLimitPair twistLimit(-PxPi / 4.f, PxPi / 4.f);
		static bool isTwistLimit = false;
		ImGui::Text("_Twsit Limit (Radian)");
		ImGui::Checkbox("_Twist Limit", &isTwistLimit);
		ImGui::DragFloat("_Lower Limit Angle", &twistLimit.lower, 0.001f);
		ImGui::DragFloat("_Upper Limit Angle", &twistLimit.upper, 0.001f);

		static PxJointLimitCone swingLimit(-PxPi / 4.f, PxPi / 4.f);
		static bool isSwingLimit = false;
		ImGui::Text("_Swing Limit (Radian)");
		ImGui::Checkbox("_Swing Limit", &isSwingLimit);
		ImGui::DragFloat("_Y Limit Angle", &swingLimit.yAngle, 0.001f);
		ImGui::DragFloat("_Z Limit Angle", &swingLimit.zAngle, 0.001f);

		ImGui::Text("Motion");
		for (int i = 0; i < 6; ++i)
			ImGui::Combo(labels[i], &motionVal[i], motionStr);

		static bool drives[6] = { 0, 0, 0, 0, 0, 0 };
		static PxD6JointDrive drive(10.f, -20.f, PX_MAX_F32, true);
		static bool driveAccel = true;
		ImGui::Text("Drive");
		ImGui::DragFloat("Drive Stiffness", &drive.stiffness);
		ImGui::DragFloat("Drive Dampping", &drive.damping);
		ImGui::DragFloat("Drive Force Limit", &drive.forceLimit);
		ImGui::Checkbox("Drive Accel", &driveAccel);

		ImGui::Checkbox(labels2[0], &drives[0]);
		ImGui::SameLine();
		ImGui::Checkbox(labels2[1], &drives[1]);
		ImGui::SameLine();
		ImGui::Checkbox(labels2[2], &drives[2]);

		ImGui::Checkbox(labels2[3], &drives[3]);
		ImGui::SameLine();
		ImGui::Checkbox(labels2[4], &drives[4]);
		ImGui::SameLine();
		ImGui::Checkbox(labels2[5], &drives[5]);

		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.6f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.8f, 0.1f, 1.f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.3f, 0.3f, 0.1f, 1.f));
		if (ImGui::Button("Create D6 Joint"))
		{
			sSpawnObj result = SpawnObject();
			if (result.node0)
			{
				const Vector3 center = (m_pos0 + m_pos1) * 0.5f;
				const Vector3 p0 = center - m_pos0;
				const Vector3 p1 = center - m_pos1;

				PxD6Joint *joint = PxD6JointCreate(*g_physics.m_physics
					, result.actor0, PxTransform(*(PxVec3*)&p0)
					, result.actor1, PxTransform(*(PxVec3*)&p1));

				if (isLinearLimit)
					joint->setLinearLimit(linearLimit);
				if (isTwistLimit)
					joint->setTwistLimit(twistLimit);
				if (isSwingLimit)
					joint->setSwingLimit(swingLimit);

				for (int i = 0; i < 6; ++i)
					joint->setMotion((PxD6Axis::Enum)i, (PxD6Motion::Enum)motionVal[i]);

				for (int i = 0; i < 6; ++i)
					if (drives[i])
						joint->setDrive((PxD6Drive::Enum)i, drive);

				result.actor0->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic0);
				result.actor1->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, m_isKinematic1);

				g_physics.m_actors.push_back({ result.actor0, "cube", result.node0 });
				g_physics.m_actors.push_back({ result.actor1, "cube", result.node1 });
				g_physics.AddJoint(joint);
			}
		}
		ImGui::PopStyleColor(3);

		ImGui::Spacing();
		ImGui::Spacing();
	}
}


// calc localFrame for PxJoint~ Function seriese
// worldTm0 : actor0 world transform
// worldTm1 : actor1 world transform
// revoluteAxis : revolution Axis
//			      if ZeroVector, ignore revolute axis
// out0 : actor0 localFrame
// out1 : actor1 localFrame
void cInformationView::GetLocalFrame(const Transform &worldTm0, const Transform &worldTm1
	, const Vector3 &revoluteAxis
	, OUT physx::PxTransform &out0, OUT physx::PxTransform &out1)

{
	using namespace physx;

	Transform tfm0 = worldTm0;
	Transform tfm1 = worldTm1;

	const Vector3 center = (tfm0.pos + tfm1.pos) * 0.5f;
	Vector3 p0 = center - tfm0.pos;
	Vector3 p1 = center - tfm1.pos;
	Quaternion q0 = tfm0.rot.Inverse();
	Quaternion q1 = tfm1.rot.Inverse();

	if (revoluteAxis != Vector3::Zeroes)
	{
		Quaternion rot(revoluteAxis, Vector3(1, 0, 0));
		tfm0.rot *= rot;
		tfm1.rot *= rot;

		p0 = (tfm0.pos - center) * rot + center;
		p1 = (tfm1.pos - center) * rot + center;
		p0 = center - p0;
		p1 = center - p1;
		q0 = tfm0.rot.Inverse();
		q1 = tfm1.rot.Inverse();
	}

	const PxTransform localFrame0 = PxTransform(*(PxQuat*)&q0) * PxTransform(*(PxVec3*)&p0);
	const PxTransform localFrame1 = PxTransform(*(PxQuat*)&q1) * PxTransform(*(PxVec3*)&p1);

	out0 = localFrame0;
	out1 = localFrame1;
}
