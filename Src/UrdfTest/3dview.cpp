
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

	m_art.Clear(&m_physics);
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
	m_physSync->SpawnPlane(&renderer, Vector3(0, 1, 0));

	InitRobot1();
	
	//m_box.Create(renderer);
	//m_box.SetCube(Transform(Vector3(0, 0, 0), Vector3::Ones*0.5f));
	//m_cylinder.Create(renderer, 1.f, 1.f, 10, (eVertexType::POSITION | eVertexType::NORMAL | eVertexType::COLOR)
	//, cColor::WHITE, eCylinderType::AxisY);
	//m_sphere.Create(renderer, 1, 20, 20);

	return true;
}


// set articulation joint drive parameter
bool SetJointDriveParam(physx::PxArticulationJointReducedCoordinate* joint, const float velocity)
{
	using namespace physx;
	PxArticulationDrive posDrive;
	posDrive.stiffness = 100.f;
	posDrive.damping = 0.f;
	posDrive.maxForce = 100.f;
	posDrive.driveType = PxArticulationDriveType::eACCELERATION;

	joint->setDriveParams(PxArticulationAxis::eTWIST, posDrive);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
	joint->setDriveVelocity(PxArticulationAxis::eTWIST, velocity);
	return true;
}


// initialize robot articulation
void c3DView::InitRobot1()
{
	using namespace physx;

	if (!m_urdf.Open("robot.urdf"))
		return;

	const Transform tfm(Vector3(0,1,0));

	phys::cPhysicsEngine& physics = m_physics;

	physx::PxArticulationJointReducedCoordinate* joint = nullptr;

	physx::PxMaterial* material = physics.s_physics->createMaterial(0.5f, 0.5f, 0.1f); // wheel material
	physx::PxMaterial* wheelMtrl = physics.s_physics->createMaterial(1.0f, 1.0f, 0.1f); // wheel material

	m_art.Create(physics, false, 32);

	map<string, int> ids; // key: link name, value: physx object id
	map<string, Matrix44> visWTms; // key: link name, value: visual world matrix
	map<string, string> parents; // key: child link name, value: parent link name

	// collect all visual world transform
	m_urdf.Traverse(
		[&](rik::cNode2* node, const Matrix44& tm) {
			if (1 == node->m_nodeType) // link
			{
				visWTms.insert({ node->m_name, tm });
			}
			else if (3 == node->m_nodeType) // visual
			{
				rik::cVisual2* vis = dynamic_cast<rik::cVisual2*>(node);
				if (!vis) return false; // error, ignore
				rik::cLink2* link = dynamic_cast<rik::cLink2*>(vis->m_parent);
				if (!link) return false; // error, ignore

				visWTms.insert({ link->m_name, tm });
			}
			else if (2 == node->m_nodeType) // joint
			{
				rik::cJoint2* joint = dynamic_cast<rik::cJoint2*>(node);
				if (!joint) return false; // error, ignore

				parents.insert({ joint->m_childLink, joint->m_parentLink });
			}
			return false; // loop all
		});

	// spawn physx object
	m_urdf.Traverse(
		[&](rik::cNode2* node, const Matrix44& tm) {
			if (3 != node->m_nodeType) // only visual
				return false; // all loop

			rik::cVisual2* vis = dynamic_cast<rik::cVisual2*>(node);
			if (!vis) return false; // error, ignore
			rik::cLink2* link = dynamic_cast<rik::cLink2*>(vis->m_parent);
			if (!link) return false; // error, ignore

			const float mass = 1.f;
			int parentId = -1; // parent link physx object Id, -1:root
			Matrix44 ptm; // parent tm

			// find parent link
			auto it = parents.find(link->m_name);
			if (parents.end() != it)
			{
				const string& parentLinkName = it->second;

				// find parent tm
				auto it2 = visWTms.find(parentLinkName);
				if (visWTms.end() == it2) return false; // error return

				auto it3 = ids.find(parentLinkName);

				ptm = it2->second;
				parentId = (ids.end() == it3) ? -1 : it3->second;
			}

			Transform linkTfm;
			linkTfm.pos = tm.GetPosition();// .ToOpenGL();
			linkTfm.rot = tm.GetQuaternion();// .ToOpenGL();

			int linkId = -1;
			switch (vis->m_geoType)
			{
			case 0: // box
				linkTfm.scale = vis->m_dimension;
				linkId = m_art.AddBoxLink(physics, parentId, linkTfm, mass);
				break;

			case 1: // sphere
				linkId = m_art.AddSphereLink(physics, parentId, linkTfm
					, vis->m_radius, mass);
				break;

			case 2: // cylinder
				linkId = m_art.AddCylinderLink(physics, parentId, linkTfm
					, vis->m_radius, vis->m_height, mass);
				break;
			}

			if (linkId < 0)
				return false; // error return

			ids.insert({ link->m_name, linkId });

			return false; // loop all
		});

	// spawn physx joint
	m_urdf.Traverse(
		[&](rik::cNode2* node, const Matrix44& tm) {
			if (2 != node->m_nodeType) // only joint
				return false; // all loop

			rik::cJoint2* joint = dynamic_cast<rik::cJoint2*>(node);
			if (!joint) return false; // error, ignore

			auto it1 = ids.find(joint->m_childLink);
			if (ids.end() == it1) return  false; // error return
			auto it2 = visWTms.find(joint->m_parentLink);
			if (visWTms.end() == it2) return  false; // error return
			auto it3 = visWTms.find(joint->m_childLink);
			if (visWTms.end() == it2) return  false; // error return

			const int childLinkId = it1->second;

			const Matrix44& ptm = it2->second; // parent transform
			const Matrix44& ctm = it3->second; // child transform
			const Vector3 pos0 = ptm.GetPosition();// .ToOpenGL();
			const Vector3 pos1 = ctm.GetPosition();// .ToOpenGL();

			physx::PxArticulationJointReducedCoordinate* j = nullptr;
			switch (joint->m_type)
			{
			case 0: // revolute
				j = m_art.AddJoint(childLinkId, phys::eJointType::Revolute
					, pos0, pos1, joint->m_axis);
				m_art.SetJointDriveVelocity(j, 0.f, 1000.f, 1000.f, 1.f); // default
				//m_art.SetJointDriveTarget(joint, 1000.f, 0.f, 1000.f, 0.f);
				break;
			case 1: // prismatic
				break;
			case 3: // universal
				break;
			case 4: // fixed
				j = m_art.AddJoint(childLinkId, phys::eJointType::Fixed
					, pos0, pos1, joint->m_axis);
				break;
			default:
				break;
			}

			return false; // loop all
		});

	if (m_art.m_links.empty())
		return; // nothing to do

	Transform tm = tfm;
	m_art.SetGlobalPose(tm);
	m_art.AddScene(physics);
	material->release();
	wheelMtrl->release();
}


// render robot
void c3DView::RenderUrdf(graphic::cRenderer& renderer, const float deltaSeconds)
{
	RenderUrdfNode(renderer, m_urdf.m_root, Matrix44::Identity.GetMatrixXM(), 1);

	CommonStates state(renderer.GetDevice());
	renderer.GetDevContext()->OMSetDepthStencilState(state.DepthNone(), 0);
	RenderUrdfNode(renderer, m_urdf.m_root, Matrix44::Identity.GetMatrixXM(), 2);
	renderer.GetDevContext()->OMSetDepthStencilState(state.DepthDefault(), 0);
}


// render robot node
bool c3DView::RenderUrdfNode(graphic::cRenderer& renderer
	, rik::cNode2* node, const XMMATRIX& parentTm
	, const int flags //= 0
)
{
	RETV(!node, false);

	switch (node->m_nodeType)
	{
	case 0: // node
		for (auto& p : node->m_children)
			RenderUrdfNode(renderer, p, parentTm, flags);
		break;
	case 1: // link
	{
		const XMMATRIX tm = node->m_transform.GetMatrixXM() * parentTm;
		for (auto& p : node->m_children)
			RenderUrdfNode(renderer, p, tm, flags);
	}
	break;
	case 2: // joint
		if (rik::cJoint2* joint = dynamic_cast<rik::cJoint2*>(node))
			RenderUrdfJoint(renderer, joint, parentTm, flags);
		break;
	case 3: // visual
		if (rik::cVisual2* visual = dynamic_cast<rik::cVisual2*>(node))
			RenderUrdfVisual(renderer, visual, parentTm, flags);
		break;
	default: assert(0); break;
	}
	return true;
}


// render joint
bool c3DView::RenderUrdfJoint(graphic::cRenderer& renderer
	, rik::cJoint2* joint, const XMMATRIX& parentTm
	, const int flags //= 0
)
{
	XMMATRIX tm;
	switch (joint->m_type)
	{
	case 0: // revolute
	{
		Transform tfm = joint->m_transform;
		const Quaternion rot(joint->m_axis, joint->m_val);
		tfm.rot = rot * tfm.rot;
		tm = tfm.GetMatrixXM() * parentTm;

		if ((0 == flags) || (2 == flags))
		{
			// render joint cylinder
			if (!m_cylinerLine.m_lines.m_vtxBuff.m_vtxBuff)
				m_cylinerLine.Create(renderer, 0.025f, 0.05f, 30);

			m_cylinerLine.SetColor(graphic::cColor::GREEN);
			m_cylinerLine.SetPos(Vector3());// tfm.pos);
			Quaternion rot;
			rot.SetRotationArc(Vector3(0, 1, 0), joint->m_axis);
			m_cylinerLine.SetRotation(rot);

			// render joint axis
			const float length = 0.1f;
			renderer.m_dbgLine.SetColor(graphic::cColor::GREEN);
			//renderer.m_dbgLine.SetLine(tfm.pos, tfm.pos + joint->m_axis * length, 0.001f);
			renderer.m_dbgLine.SetLine(Vector3(), joint->m_axis * length, 0.001f);
			renderer.m_dbgLine.Render(renderer, tm);// parentTm);

			m_cylinerLine.Render(renderer, tm);// parentTm);
		}
	}
	break;

	default:
	{
		// render default joint geometry
		if ((0 == flags) || (2 == flags))
		{
			Transform tfm = joint->m_transform;
			tfm.scale = Vector3(0.02f, 0.02f, 0.02f);
			renderer.m_dbgBox.SetBox(tfm);
			renderer.m_dbgBox.SetColor(graphic::cColor::GREEN);
			renderer.m_dbgBox.Render(renderer, parentTm);
		}

		Transform tfm = joint->m_transform;
		tfm.rot.SetIdentity(); // ignore rotation
		tm = tfm.GetMatrixXM() * parentTm;
	}
	break;
	}

	//m_tm = tm; // update world transform
	for (auto& p : joint->m_children)
		RenderUrdfNode(renderer, p, tm, flags);
	return true;
}


// render visual
bool c3DView::RenderUrdfVisual(graphic::cRenderer& renderer
	, rik::cVisual2* visual, const XMMATRIX& parentTm
	, const int flags //= 0
)
{
	if ((0 == flags) || (1 == flags))
	{
		switch (visual->m_geoType)
		{
		case 0: // box
		{
			if (!m_box.m_lines.m_vtxBuff.m_vtxBuff)
				m_box.Create(renderer);
			if (!m_box1.m_shape.m_vtxBuff.m_vtxBuff)
				m_box1.Create(renderer);

			{
				Transform tfm = visual->m_transform;
				tfm.scale = visual->m_dimension * 0.5f;
				m_box.SetBox(tfm);
			}
			{
				Transform tfm = visual->m_transform;
				tfm.scale = visual->m_dimension * 0.5f;
				m_box1.SetCube(tfm);
			}

			m_box1.SetColor(visual->m_color);
			m_box1.Render(renderer, parentTm);
			m_box.SetColor(graphic::cColor(1.f, 1.f, 1.f, 0.65f));
			m_box.Render(renderer, parentTm);
		}
		break;

		case 1: // sphere
		{
			if (!m_sphere.m_lines.m_vtxBuff.m_vtxBuff)
				m_sphere.Create(renderer, visual->m_radius, 30);

			m_sphere.SetPos(visual->m_transform.pos);
			m_sphere.SetColor(visual->m_color);
			m_sphere.SetRadius(visual->m_radius);
			m_sphere.Render(renderer, parentTm);
		}
		break;

		case 2: // cylinder
		{
			if (!m_cylinder.m_lines.m_vtxBuff.m_vtxBuff)
				m_cylinder.Create(renderer, visual->m_radius, visual->m_height, 30);

			m_cylinder.m_transform.pos = visual->m_transform.pos;
			m_cylinder.m_transform.rot = visual->m_transform.rot;
			m_cylinder.SetColor(visual->m_color);
			m_cylinder.Render(renderer, parentTm);
		}
		break;

		default:
			assert(0);
			break;
		}
	}

	const XMMATRIX tm = visual->m_transform.GetMatrixXM() * parentTm;
	for (auto& p : visual->m_children)
		RenderUrdfNode(renderer, p, tm, flags);
	return true;
}


void c3DView::OnUpdate(const float deltaSeconds)
{
	if (m_isSimulation)
	{
		using namespace physx;

		static bool toggle = true;
		static float incT = 0.f;
		incT += deltaSeconds;
		if (incT > 1.f)
		{
			incT = 0.f;
			const float target = MATH_PI / 8.f;

			if (m_motorJoint1)
				m_motorJoint1->setDriveTarget(PxArticulationAxis::eTWIST, toggle ? target : -target);
			if (m_motorJoint2)
				m_motorJoint2->setDriveTarget(PxArticulationAxis::eTWIST, toggle ? target : -target);
			if (m_motorJoint3)
				m_motorJoint3->setDriveTarget(PxArticulationAxis::eTWIST, toggle ? target : -target);
			if (m_motorJoint4)
				m_motorJoint4->setDriveTarget(PxArticulationAxis::eTWIST, toggle ? target : -target);

			toggle = !toggle;
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

		m_art.Render(renderer);
		RenderUrdf(renderer, deltaSeconds);

		//m_box.SetColor(cColor::GREEN);
		//for (auto& pos : m_jointPoss)
		//{
		//	Transform tm(pos, Vector3::Ones * 0.01f);
		//	m_box.SetCube(tm);
		//	m_box.Render(renderer);
		//}

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
