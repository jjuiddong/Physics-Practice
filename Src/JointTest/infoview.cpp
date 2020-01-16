
#include "stdafx.h"
#include "infoview.h"


cInformationView::cInformationView(const StrId &name)
	: framework::cDockWindow(name)
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
	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Spacing();

	RenderFixedJoint();
	RenderSphericalJoint();
	RenderRevoluteJoint();
	RenderPrismaticJoint();
	RenderDistanceJoint();
	RenderD6Joint();
}


void cInformationView::RenderFixedJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Fixed Joint"))
	{
		static float distance = 1.f;
		static float scale = 0.5f;
		static int item = 0;
		ImGui::Combo("Model F", &item, "Box\0Sphere\0Capsule\0\0");
		ImGui::DragFloat("Distance F", &distance, 0.01f, 0, 100.f);
		ImGui::DragFloat("Scale F", &scale, 0.01f, 0, 100.f);

		const Vector3 pos0(distance, 10, 0);
		const Vector3 pos1(-distance, 10, 0);

		if (ImGui::Button("Create Fixed Joint"))
		{
			graphic::cNode *node0 = nullptr, *node1 = nullptr;
			PxRigidDynamic *actor0 = nullptr, *actor1 = nullptr;
			switch (item)
			{
			case 0:
				node0 = g_physics.SpawnBox(renderer, pos0, Vector3::Ones * scale);
				node1 = g_physics.SpawnBox(renderer, pos1, Vector3::Ones * scale);
				actor0 = g_physics.CreateBox(pos0, Vector3::Ones * scale);
				actor1 = g_physics.CreateBox(pos1, Vector3::Ones * scale);
				break;

			case 1:
				node0 = g_physics.SpawnSphere(renderer, pos0, scale);
				node1 = g_physics.SpawnSphere(renderer, pos1, scale);
				actor0 = g_physics.CreateSphere(pos0, scale);
				actor1 = g_physics.CreateSphere(pos1, scale);
				break;

			case 2:
				node0 = g_physics.SpawnCapsule(renderer, pos0, scale, scale*2.f);
				node1 = g_physics.SpawnCapsule(renderer, pos1, scale, scale*2.f);
				actor0 = g_physics.CreateCapsule(pos0, scale, scale*2.f);
				actor1 = g_physics.CreateCapsule(pos1, scale, scale*2.f);
				break;
			}

			if (node0)
			{
				PxFixedJoint *joint = PxFixedJointCreate(*g_physics.m_physics
					, actor0, PxTransform(PxVec3(-distance,0,0))
					, actor1, PxTransform(PxVec3(distance, 0,0)));

				g_physics.m_actors.push_back({ actor0, "cube", node0 });
				g_physics.m_actors.push_back({ actor1, "cube", node1 });
				g_physics.AddJoint(joint);
			}
		}

		ImGui::Spacing();
		ImGui::Spacing();
	}
}


void cInformationView::RenderSphericalJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Spherical Joint"))
	{
		static float distance = 1.f;
		static float scale = 0.5f;
		static int item = 0;
		ImGui::Combo("Model S", &item, "Box\0Sphere\0Capsule\0\0");
		ImGui::DragFloat("Distance S", &distance, 0.01f, 0, 100.f);
		ImGui::DragFloat("Scale S", &scale, 0.01f, 0, 100.f);

		const Vector3 pos0(distance, 10, 0);
		const Vector3 pos1(-distance, 10, 0);

		if (ImGui::Button("Create Sperical Joint"))
		{
			graphic::cNode *node0 = nullptr, *node1 = nullptr;
			PxRigidDynamic *actor0 = nullptr, *actor1 = nullptr;
			switch (item)
			{
			case 0:
				node0 = g_physics.SpawnBox(renderer, pos0, Vector3::Ones * scale);
				node1 = g_physics.SpawnBox(renderer, pos1, Vector3::Ones * scale);
				actor0 = g_physics.CreateBox(pos0, Vector3::Ones * scale);
				actor1 = g_physics.CreateBox(pos1, Vector3::Ones * scale);
				break;

			case 1:
				node0 = g_physics.SpawnSphere(renderer, pos0, scale);
				node1 = g_physics.SpawnSphere(renderer, pos1, scale);
				actor0 = g_physics.CreateSphere(pos0, scale);
				actor1 = g_physics.CreateSphere(pos1, scale);
				break;

			case 2:
				node0 = g_physics.SpawnCapsule(renderer, pos0, scale, scale*2.f);
				node1 = g_physics.SpawnCapsule(renderer, pos1, scale, scale*2.f);
				actor0 = g_physics.CreateCapsule(pos0, scale, scale*2.f);
				actor1 = g_physics.CreateCapsule(pos1, scale, scale*2.f);
				break;
			}

			if (node0)
			{
				PxSphericalJoint *joint = PxSphericalJointCreate(*g_physics.m_physics
					, actor0, PxTransform(PxVec3(-distance, 0, 0))
					, actor1, PxTransform(PxVec3(distance, 0, 0)));

				joint->setLimitCone(PxJointLimitCone(PxPi / 2, PxPi / 6, 0.01f));
				joint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);

				g_physics.m_actors.push_back({ actor0, "cube", node0 });
				g_physics.m_actors.push_back({ actor1, "cube", node1 });
				g_physics.AddJoint(joint);
			}
		}

		ImGui::Spacing();
		ImGui::Spacing();
	}
}


void cInformationView::RenderRevoluteJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Revolute Joint"))
	{
		static float distance = 1.f;
		static float scale = 0.5f;
		static int item = 0;
		ImGui::Combo("Model R", &item, "Box\0Sphere\0Capsule\0\0");
		ImGui::DragFloat("Distance R", &distance, 0.01f, 0, 100.f);
		ImGui::DragFloat("Scale R", &scale, 0.01f, 0, 100.f);

		const Vector3 pos0(distance, 10, 0);
		const Vector3 pos1(-distance, 10, 0);

		if (ImGui::Button("Create Revolute Joint"))
		{
			graphic::cNode *node0 = nullptr, *node1 = nullptr;
			PxRigidDynamic *actor0 = nullptr, *actor1 = nullptr;
			switch (item)
			{
			case 0:
				node0 = g_physics.SpawnBox(renderer, pos0, Vector3::Ones * scale);
				node1 = g_physics.SpawnBox(renderer, pos1, Vector3::Ones * scale);
				actor0 = g_physics.CreateBox(pos0, Vector3::Ones * scale);
				actor1 = g_physics.CreateBox(pos1, Vector3::Ones * scale);
				break;

			case 1:
				node0 = g_physics.SpawnSphere(renderer, pos0, scale);
				node1 = g_physics.SpawnSphere(renderer, pos1, scale);
				actor0 = g_physics.CreateSphere(pos0, scale);
				actor1 = g_physics.CreateSphere(pos1, scale);
				break;

			case 2:
				node0 = g_physics.SpawnCapsule(renderer, pos0, scale, scale*2.f);
				node1 = g_physics.SpawnCapsule(renderer, pos1, scale, scale*2.f);
				actor0 = g_physics.CreateCapsule(pos0, scale, scale*2.f);
				actor1 = g_physics.CreateCapsule(pos1, scale, scale*2.f);
				break;
			}

			if (node0)
			{
				PxRevoluteJoint *joint = PxRevoluteJointCreate(*g_physics.m_physics
					, actor0, PxTransform(PxVec3(-distance, 0, 0))
					, actor1, PxTransform(PxVec3(distance, 0, 0)));

				joint->setLimit(PxJointAngularLimitPair(-PxPi / 4, PxPi / 4, 0.01f));
				joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);

				g_physics.m_actors.push_back({ actor0, "cube", node0 });
				g_physics.m_actors.push_back({ actor1, "cube", node1 });
				g_physics.AddJoint(joint);
			}
		}

		ImGui::Spacing();
		ImGui::Spacing();
	}

}


void cInformationView::RenderPrismaticJoint()
{
	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Prismatic Joint"))
	{
		if (ImGui::Button("Create"))
		{

		}
	}
}


void cInformationView::RenderDistanceJoint()
{
	using namespace physx;
	graphic::cRenderer &renderer = GetRenderer();

	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("Distance Joint"))
	{
		static float distance = 1.f;
		static float scale = 0.5f;
		static int item = 0;
		ImGui::Combo("Model D", &item, "Box\0Sphere\0Capsule\0\0");
		ImGui::DragFloat("Distance D", &distance, 0.01f, 0, 100.f);
		ImGui::DragFloat("Scale D", &scale, 0.01f, 0, 100.f);

		const Vector3 pos0(distance, 10, 0);
		const Vector3 pos1(-distance, 10, 0);

		if (ImGui::Button("Create Distance Joint"))
		{
			graphic::cNode *node0 = nullptr, *node1 = nullptr;
			PxRigidDynamic *actor0 = nullptr, *actor1 = nullptr;
			switch (item)
			{
			case 0:
				node0 = g_physics.SpawnBox(renderer, pos0, Vector3::Ones * scale);
				node1 = g_physics.SpawnBox(renderer, pos1, Vector3::Ones * scale);
				actor0 = g_physics.CreateBox(pos0, Vector3::Ones * scale);
				actor1 = g_physics.CreateBox(pos1, Vector3::Ones * scale);
				break;

			case 1:
				node0 = g_physics.SpawnSphere(renderer, pos0, scale);
				node1 = g_physics.SpawnSphere(renderer, pos1, scale);
				actor0 = g_physics.CreateSphere(pos0, scale);
				actor1 = g_physics.CreateSphere(pos1, scale);
				break;

			case 2:
				node0 = g_physics.SpawnCapsule(renderer, pos0, scale, scale*2.f);
				node1 = g_physics.SpawnCapsule(renderer, pos1, scale, scale*2.f);
				actor0 = g_physics.CreateCapsule(pos0, scale, scale*2.f);
				actor1 = g_physics.CreateCapsule(pos1, scale, scale*2.f);
				break;
			}

			if (node0)
			{
				PxDistanceJoint *joint = PxDistanceJointCreate(*g_physics.m_physics
					, actor0, PxTransform(PxVec3(-distance, 0, 0))
					, actor1, PxTransform(PxVec3(distance, 0, 0)));

				joint->setMaxDistance(distance);
				joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);

				g_physics.m_actors.push_back({ actor0, "cube", node0 });
				g_physics.m_actors.push_back({ actor1, "cube", node1 });
				g_physics.AddJoint(joint);
			}
		}

		ImGui::Spacing();
		ImGui::Spacing();
	}

}


void cInformationView::RenderD6Joint()
{
	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
	if (ImGui::CollapsingHeader("D6 Joint"))
	{
		if (ImGui::Button("Create"))
		{

		}
	}

}
