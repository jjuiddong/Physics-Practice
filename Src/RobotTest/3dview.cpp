
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

	//InitRobot1();
	InitRobot2();
	
	m_box.Create(renderer);
	m_box.SetCube(Transform(Vector3(0, 0, 0), Vector3::Ones*0.5f));
	m_cylinder.Create(renderer, 1.f, 1.f, 10);
	m_sphere.Create(renderer, 1, 20, 20);

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
	posDrive.driveType = PxArticulationDriveType::eVELOCITY;

	joint->setDriveParams(PxArticulationAxis::eTWIST, posDrive);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
	joint->setDriveVelocity(PxArticulationAxis::eTWIST, velocity);
	return true;
}


// initialize robot articulation
void c3DView::InitRobot1()
{
	using namespace physx;

	const float frame_y = 1.0f;
	const float frame_cx = 1.0f; // frame x-axis size
	const float frame_cz = 1.0f; // frame z-axis size
	const float frame_h = 0.2f; // frame height
	const float frame_mass = 1.0f;
	const float wheel_r = 0.2f;
	const float wheel_d = 0.1f;
	const float wheel_gap = wheel_d/2 + 0.1f;
	const float wheel_mass = 1.0f;

	m_art.Create(m_physics, false, 32);

	physx::PxArticulationJointReducedCoordinate *joint = nullptr;

	// create frame...
	const Transform frameTfm(Vector3(0.f, frame_y, 0.f), Vector3(frame_cx/2, frame_h/2, frame_cz/2));
	const int frame = m_art.AddBoxLink(m_physics, -1, frameTfm, frame_mass);

	// create wheel
	const Transform wheelTfm1(Vector3(frame_cx/2 + wheel_gap, frame_y, frame_cz/2));
	const int wheel1 = m_art.AddCylinderLink(m_physics, frame, wheelTfm1, wheel_r, wheel_d);

	const Transform wheelTfm2(Vector3(-frame_cx / 2 - wheel_gap, frame_y, frame_cz / 2));
	const int wheel2 = m_art.AddCylinderLink(m_physics, frame, wheelTfm2, wheel_r, wheel_d);

	const Transform wheelTfm3(Vector3(frame_cx / 2 + wheel_gap, frame_y, -frame_cz / 2));
	const int wheel3 = m_art.AddCylinderLink(m_physics, frame, wheelTfm3, wheel_r, wheel_d);

	const Transform wheelTfm4(Vector3(-frame_cx / 2 - wheel_gap, frame_y, -frame_cz / 2));
	const int wheel4 = m_art.AddCylinderLink(m_physics, frame, wheelTfm4, wheel_r, wheel_d);

	joint = m_art.AddJoint(wheel1, phys::eJointType::Revolute
		, Vector3(frame_cx/2, frame_y, frame_cz/2), wheelTfm1.pos);
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, 10.f);

	joint = m_art.AddJoint(wheel2, phys::eJointType::Revolute
		, Vector3(-frame_cx / 2, frame_y, frame_cz / 2), wheelTfm2.pos);
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, -10.f);

	joint = m_art.AddJoint(wheel3, phys::eJointType::Revolute
		, Vector3(frame_cx / 2, frame_y, -frame_cz / 2), wheelTfm3.pos);
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, 10.f);

	joint = m_art.AddJoint(wheel4, phys::eJointType::Revolute
		, Vector3(-frame_cx / 2, frame_y, -frame_cz / 2), wheelTfm4.pos);
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, -10.f);

	m_art.AddScene(m_physics);
}


// initialize 4wheel steer robot
void c3DView::InitRobot2()
{
	using namespace physx;

	const float frame_y = 1.0f;
	const float frame_cx = 1.0f; // frame x-axis size
	const float frame_cz = 1.0f; // frame z-axis size
	const float frame_h = 0.2f; // frame height
	const float frame_mass = 1.0f;

	const float motor_cx = 0.3f;
	const float motor_cz = 0.2f;
	const float motor_h = 0.2f;
	const float motor_y = frame_y - frame_h / 2 - motor_h / 2 - 0.15f;
	const float motor_mass = 1.f;

	const float wheel_r = 0.2f;
	const float wheel_d = 0.2f;
	const float wheel_gap = motor_cx/2 + wheel_d/2 + 0.1f;
	const float wheel_y = motor_y;
	const float wheel_mass = 1.0f;

	const float velocity = 1.f;

	m_physics.m_material->setDynamicFriction(1.0f);
	m_physics.m_material->setStaticFriction(1.0f);

	m_art.Create(m_physics, false, 32);

	physx::PxArticulationJointReducedCoordinate* joint = nullptr;

	// create frame...
	const Transform frameTfm(Vector3(0.f, frame_y, 0.f), Vector3(frame_cx / 2, frame_h / 2, frame_cz / 2));
	const int frame = m_art.AddBoxLink(m_physics, -1, frameTfm, frame_mass);

	// create motor
	const Vector3 motorScale(motor_cx / 2, motor_h / 2, motor_cz / 2);

	const Transform motorTfm1(Vector3(frame_cx / 2, motor_y, frame_cz / 2), motorScale);
	const int motor1 = m_art.AddBoxLink(m_physics, frame, motorTfm1, motor_mass);

	const Transform motorTfm2(Vector3(-frame_cx / 2, motor_y, frame_cz / 2), motorScale);
	const int motor2 = m_art.AddBoxLink(m_physics, frame, motorTfm2, motor_mass);

	const Transform motorTfm3(Vector3(frame_cx / 2, motor_y, -frame_cz / 2), motorScale);
	const int motor3 = m_art.AddBoxLink(m_physics, frame, motorTfm3, motor_mass);

	const Transform motorTfm4(Vector3(-frame_cx / 2, motor_y, -frame_cz / 2), motorScale);
	const int motor4 = m_art.AddBoxLink(m_physics, frame, motorTfm4, motor_mass);

	joint = m_art.AddJoint(motor1, phys::eJointType::Revolute
		, Vector3(frame_cx / 2, wheel_y, frame_cz / 2), motorTfm1.pos, Vector3(0,1,0));
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, 0.f);
	m_motorJoint1 = joint;

	joint = m_art.AddJoint(motor2, phys::eJointType::Revolute
		, Vector3(-frame_cx / 2, wheel_y, frame_cz / 2), motorTfm2.pos, Vector3(0, 1, 0));
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, 0.f);
	m_motorJoint2 = joint;

	joint = m_art.AddJoint(motor3, phys::eJointType::Revolute
		, Vector3(frame_cx / 2, wheel_y, -frame_cz / 2), motorTfm3.pos, Vector3(0, 1, 0));
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, 0.f);
	m_motorJoint3 = joint;

	joint = m_art.AddJoint(motor4, phys::eJointType::Revolute
		, Vector3(-frame_cx / 2, wheel_y, -frame_cz / 2), motorTfm4.pos, Vector3(0, 1, 0));
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, 0.f);
	m_motorJoint4 = joint;


	// create wheel
	const Transform wheelTfm1(motorTfm1.pos + Vector3(wheel_gap, 0, 0));
	const int wheel1 = m_art.AddCylinderLink(m_physics, motor1, wheelTfm1, wheel_r, wheel_d, wheel_mass);

	const Transform wheelTfm2(motorTfm2.pos + Vector3(-wheel_gap, 0, 0));
	const int wheel2 = m_art.AddCylinderLink(m_physics, motor2, wheelTfm2, wheel_r, wheel_d, wheel_mass);

	const Transform wheelTfm3(motorTfm3.pos + Vector3(wheel_gap, 0, 0));
	const int wheel3 = m_art.AddCylinderLink(m_physics, motor3, wheelTfm3, wheel_r, wheel_d, wheel_mass);

	const Transform wheelTfm4(motorTfm4.pos + Vector3(-wheel_gap, 0, 0));
	const int wheel4 = m_art.AddCylinderLink(m_physics, motor4, wheelTfm4, wheel_r, wheel_d, wheel_mass);

	joint = m_art.AddJoint(wheel1, phys::eJointType::Revolute, motorTfm1.pos, wheelTfm1.pos);
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, velocity);

	joint = m_art.AddJoint(wheel2, phys::eJointType::Revolute, motorTfm2.pos, wheelTfm2.pos);
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, velocity);

	joint = m_art.AddJoint(wheel3, phys::eJointType::Revolute, motorTfm3.pos, wheelTfm3.pos);
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, velocity);

	joint = m_art.AddJoint(wheel4, phys::eJointType::Revolute, motorTfm4.pos, wheelTfm4.pos);
	m_art.SetJointVelocityDrive(joint, 0.f, 100.f, 100.f, velocity);


	m_art.AddScene(m_physics);
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
			const float velocity = 0.5f;

			m_motorJoint1->setDriveVelocity(PxArticulationAxis::eTWIST, toggle? velocity : -velocity);
			m_motorJoint2->setDriveVelocity(PxArticulationAxis::eTWIST, toggle ? velocity : -velocity);
			m_motorJoint3->setDriveVelocity(PxArticulationAxis::eTWIST, toggle ? velocity : -velocity);
			m_motorJoint4->setDriveVelocity(PxArticulationAxis::eTWIST, toggle ? velocity : -velocity);

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

		m_box.SetColor(cColor::WHITE);

		for (auto& kv : m_art.m_links)
		{
			const phys::cArticulation::sLinkInfo& info = kv.second;
			switch (info.type)
			{
			case phys::eShapeType::Box:
			{
				const PxTransform pose = info.link->getGlobalPose();
				const Transform tm = Transform(Vector3(), info.scale) *
					Transform(*(Quaternion*)&pose.q) * Transform(*(Vector3*)&pose.p);
				m_box.SetCube(tm);
				m_box.Render(renderer);
			}
			break;

			case phys::eShapeType::Sphere:
			{
				const PxTransform pose = info.link->getGlobalPose();
				m_sphere.SetPos(*(Vector3*)&pose.p);
				m_sphere.SetRadius(info.radius);
				m_sphere.Render(renderer);
			}
			break;

			case phys::eShapeType::Capsule:
				break;

			case phys::eShapeType::Cylinder:
			{
				const PxTransform pose = info.link->getGlobalPose();
				const Transform tm = Transform(*(Quaternion*)&pose.q) * Transform(*(Vector3*)&pose.p);
				m_cylinder.m_transform.pos = tm.pos;
				m_cylinder.m_transform.rot = tm.rot;
				m_cylinder.SetDimension(info.radius, info.height);
				m_cylinder.Render(renderer);
			}
			break;

			default: break;
			}
		}

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
