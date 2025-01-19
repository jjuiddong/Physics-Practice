
#include "stdafx.h"
#include "3dview.h"

using namespace graphic;
using namespace framework;


c3DView::c3DView(const string &name)
	: framework::cDockWindow(name)
	, m_showGrid(true)
{
}

c3DView::~c3DView()
{
	m_physics.Clear();
}


bool c3DView::Init(cRenderer &renderer)
{
	const Vector3 eyePos(12.0439510f, 29.6167679f, -27.8085155f);
	const Vector3 lookAt(-10.7197342f, 0.000000000f, 47.5336151f);
	m_camera.SetCamera(eyePos, lookAt, Vector3(0, 1, 0));
	m_camera.SetProjection(MATH_PI / 4.f, m_rect.Width() / m_rect.Height(), 1.f, 1000000.f);
	m_camera.SetViewPort(m_rect.Width(), m_rect.Height());

	GetMainLight().Init(graphic::cLight::LIGHT_DIRECTIONAL);
	GetMainLight().SetDirection(Vector3(-1, -2, 1.3f).Normal());

	sf::Vector2u size((u_int)m_rect.Width() - 15, (u_int)m_rect.Height() - 50);
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

	using namespace physx;
	const float rootY = 30.f;
	Vector3 rootPoss[4] = {
		Vector3(0, rootY, 0)
		,  Vector3(0, rootY, 5)
		,  Vector3(5, rootY, 5)
		, Vector3(5, rootY, 0)
	};
	int rootIds[4];
	
	for (int i = 0; i < 4; ++i)
	{
		const int rootId0 = m_physSync->SpawnSphere(renderer, Transform(rootPoss[i]), 0.5f);
		phys::sSyncInfo *rootActor0 = m_physSync->FindSyncInfo(rootId0);
		rootActor0->actor->SetKinematic(true);
		rootIds[i] = rootId0;
	}

	m_articulation = m_physics.m_physics->createArticulation();
	m_link = m_articulation->createLink(nullptr, PxTransform(PxVec3(0, 20, 0)));
	PxRigidActorExt::createExclusiveShape(*m_link, PxBoxGeometry(PxVec3(0.5f, 0.5f, 0.5f))
		, *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*m_link, 1.0f);
	//m_articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);

	m_link2 = m_articulation->createLink(m_link, PxTransform(PxVec3(3, 18, 0)));
	PxRigidActorExt::createExclusiveShape(*m_link2, PxBoxGeometry(PxVec3(0.5f, 0.5f, 0.5f))
		, *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*m_link2, 1.0f);

	m_link3 = m_articulation->createLink(m_link2, PxTransform(PxVec3(3, 16, 0)));
	PxRigidActorExt::createExclusiveShape(*m_link3, PxBoxGeometry(PxVec3(0.5f, 0.5f, 0.5f))
		, *m_physics.m_material);
	PxRigidBodyExt::updateMassAndInertia(*m_link3, 1.0f);

	//m_link2->addForce(PxVec3(100, 0, 0), PxForceMode::eIMPULSE);

	PxArticulationJoint* joint = m_link2->getInboundJoint();
	joint->setParentPose(PxTransform(PxVec3(0,-2,0)));
	joint->setChildPose(PxTransform(PxVec3(0, +2, 0)));

	PxArticulationJoint* joint2 = m_link3->getInboundJoint();
	joint2->setParentPose(PxTransform(PxVec3(0, -2, 0)));
	joint2->setDriveType(PxArticulationJointDriveType::eTARGET);
	joint2->setTargetVelocity(PxVec3(10, 0, 0));


	//joint2->setDriveType(setsetJointType(PxArticulationJointType::eREVOLUTE);
	//joint2->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eLIMITED);
	//joint2->setLimit(PxArticulationAxis::eSWING2, -123.0f*DegToRad, 0.0f);
	//joint2->setDrive(PxArticulationAxis::eSWING2, stiffness, 0.0f, maxForce);


	m_physics.m_scene->addArticulation(*m_articulation);
	//m_articulation->setSolverIterationCounts(32);
	m_physics.m_scene->addActor(*m_link);
	m_physics.m_scene->addActor(*m_link2);

	//m_boxId = m_physSync->SpawnBox(renderer, Transform(Vector3(0, 20, 0), Vector3::Ones*0.5f));
	m_box.Create(renderer);
	m_box.SetCube(Transform(Vector3(0, 20, 0), Vector3::Ones*0.5f));



	// make rope
	int ropeIds[4];
	for (int k = 0; k < 1; ++k)
	{
		Transform prevTfm;
		prevTfm.pos = rootPoss[k];
		phys::sSyncInfo *root = m_physSync->FindSyncInfo(rootIds[k]);
		root->actor->SetKinematic(true);

		phys::cRigidActor *prev = root->actor;
		for (int i = 0; i < 0; ++i)
		{
			const Vector3 pos(rootPoss[k].x, rootY - 0.8f - i * 0.8f, rootPoss[k].z);
			
			Transform tfm;
			tfm.pos = pos;
			tfm.rot.SetRotationZ(MATH_PI / 2.f);
			const int actorId = m_physSync->SpawnCapsule(renderer, tfm, 0.1f, 0.3f, 100.f);
			phys::sSyncInfo *capsule = m_physSync->FindSyncInfo(actorId);

			//phys::cJoint *joint = new phys::cJoint();
			//joint->CreateSpherical(m_physics, prev, prevTfm, capsule->actor, tfm);
			//m_physSync->AddJoint(joint);

			prev = capsule->actor;
			prevTfm = tfm;

			if (i == 24)
				ropeIds[k] = actorId;
		}
	}

	//// make box
	//if (1)
	//{
	//	Transform tfm;
	//	tfm.pos = Vector3(2.5f, rootY - 0.8f - 25 * 0.8f - 1.5f, 2.5f);
	//	tfm.scale = Vector3::Ones * 2.7f;
	//	const int boxId = m_physSync->SpawnBox(renderer, tfm, 1.f);
	//	phys::sSyncInfo *box = m_physSync->FindSyncInfo(boxId);
	//	m_boxId = boxId;

	//	for (int i = 0; i < 4; ++i)
	//	{
	//		phys::sSyncInfo *capsule = m_physSync->FindSyncInfo(ropeIds[i]);
	//		Transform capsuleTfm = capsule->node->m_transform;

	//		phys::cJoint *joint = new phys::cJoint();
	//		//joint->CreateSpherical(m_physics, box->actor, tfm, capsule->actor, capsuleTfm);
	//		joint->CreateFixed(m_physics, box->actor, tfm, capsule->actor, capsuleTfm);
	//		m_physSync->AddJoint(joint);
	//	}
	//}

	return true;
}


void c3DView::OnUpdate(const float deltaSeconds)
{
	m_physics.PreUpdate(deltaSeconds);
}


void c3DView::OnPreRender(const float deltaSeconds)
{
	cRenderer &renderer = GetRenderer();
	cAutoCam cam(&m_camera);

	renderer.UnbindShaderAll();
	renderer.UnbindTextureAll();

	GetMainCamera().Bind(renderer);
	GetMainLight().Bind(renderer);

	m_physics.PostUpdate(deltaSeconds);

	if (m_renderTarget.Begin(renderer))
	{
		CommonStates states(renderer.GetDevice());
		renderer.GetDevContext()->RSSetState(states.CullNone());

		if (m_physSync)
		{
			for (auto &p : m_physSync->m_syncs)
				p->node->Render(renderer);
		}

		//
		physx::PxTransform pose = m_link2->getGlobalPose();
		Transform tm = Transform(*(Quaternion*)&pose.q) * Transform(*(Vector3*)&pose.p);
		m_box.SetCube(tm);
		m_box.Render(renderer);



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


// ���� �������� ��,
// ī�޶� �տ� �ڽ��� �ִٸ�, �ڽ� ���鿡�� �����.
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
	m_mousePos = mousePt;
	if (ImGui::IsMouseHoveringRect(ImVec2(-1000, -1000), ImVec2(1000, 200), false))
		return;

	if (m_mouseDown[0])
	{
		Vector3 dir = GetMainCamera().GetDirection();
		Vector3 right = GetMainCamera().GetRight();
		dir.y = 0;
		dir.Normalize();
		right.y = 0;
		right.Normalize();

		GetMainCamera().MoveRight(-delta.x * m_rotateLen * 0.001f);
		GetMainCamera().MoveFrontHorizontal(delta.y * m_rotateLen * 0.001f);
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
			// ȭ��ۿ� ���콺�� �ִ��� Capture ������ ��� Up �̺�Ʈ�� �ް��Ѵ�.
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
