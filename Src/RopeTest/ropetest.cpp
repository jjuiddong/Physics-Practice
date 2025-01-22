//
// PhysX Rope Sample
// - reference
//		https://www.youtube.com/watch?v=RFv9rC375_Q
//		https://www.youtube.com/watch?v=B7aDQyiGSPM
//
// 2025-01-19
//	- migration physx3.4 -> 5.5
//
#include "stdafx.h"
#include "3dview.h"

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

class cViewer : public framework::cGameMain2
{
public:
	cViewer();
	virtual ~cViewer();
	virtual bool OnInit() override;
	virtual void OnUpdate(const float deltaSeconds) override;
	virtual void OnRender(const float deltaSeconds) override;
	virtual void OnEventProc(const sf::Event &evt) override;
};

INIT_FRAMEWORK3(cViewer);

cPhysicsEngine g_physics;


cViewer::cViewer()
{
	m_windowName = L"Rope Test";
	m_isLazyMode = true;
	const RECT r = { 0, 0, 1024, 768 };
	//const RECT r = { 0, 0, 1280, 960 };
	m_windowRect = r;
	graphic::cResourceManager::Get()->SetMediaDirectory("./media/");
}

cViewer::~cViewer()
{
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

	if (!g_physics.InitializePhysx(m_renderer))
		return false;

	c3DView *p3dView = new c3DView("3D View");
	p3dView->Create(eDockState::DOCKWINDOW, eDockSlot::TAB, this, NULL);
	bool result = p3dView->Init(m_renderer);
	assert(result);

	m_gui.SetContext();
	m_gui.SetStyleColorsDark();

	return true;
}


void cViewer::OnUpdate(const float deltaSeconds)
{
	__super::OnUpdate(deltaSeconds);
	GetMainCamera().Update(deltaSeconds);
}


void cViewer::OnRender(const float deltaSeconds)
{
}


void cViewer::OnEventProc(const sf::Event &evt)
{
	ImGuiIO& io = ImGui::GetIO();
	switch (evt.type)
	{
	case sf::Event::KeyPressed:
		switch (evt.key.cmd) {
		case sf::Keyboard::Escape: close(); break;
		}
		break;
	}
}
