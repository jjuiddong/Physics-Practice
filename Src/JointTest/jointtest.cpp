//
// PhysX JointTest Sample
//	- reference
//		- PhysXSample code
//
#include "stdafx.h"
#include "3dview.h"
#include "infoview.h"

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
	m_windowName = L"Joint Test";
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

	cInformationView *infoView = new cInformationView("Information");
	infoView->Create(eDockState::DOCKWINDOW, eDockSlot::RIGHT, this, p3dView, 0.27f
		, framework::eDockSizingOption::PIXEL);

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
