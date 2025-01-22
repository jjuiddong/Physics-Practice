//
// 2020-01-22, jjuiddong
// 3D View
//
#pragma once


class c3DView : public framework::cDockWindow
{
public:
	c3DView(const string &name);
	virtual ~c3DView();

	bool Init(graphic::cRenderer &renderer);
	virtual void OnUpdate(const float deltaSeconds) override;
	virtual void OnRender(const float deltaSeconds) override;
	virtual void OnPreRender(const float deltaSeconds) override;
	virtual void OnResizeEnd(const framework::eDockResize::Enum type, const sRectf &rect) override;
	virtual void OnEventProc(const sf::Event &evt) override;
	virtual void OnResetDevice() override;


protected:
	void InitScissorLift1();
	void InitScissorLift2();
	void InitScissorLift3();

	physx::PxArticulationLink* CreateLink(physx::PxArticulationLink* parent
		, const Transform &tfm, const float mass = 1.f);

	physx::PxArticulationJointReducedCoordinate* CreateJoint(
		physx::PxArticulationLink* link
		, const Transform& worldTfm0, const Vector3& pivot0
		, const Transform& worldTfm1, const Vector3& pivot1
		, const physx::PxArticulationJointType::Enum jointType
	);


	void UpdateLookAt();
	void OnWheelMove(const float delta, const POINT mousePt);
	void OnMouseMove(const POINT mousePt);
	void OnMouseDown(const sf::Mouse::Button &button, const POINT mousePt);
	void OnMouseUp(const sf::Mouse::Button &button, const POINT mousePt);


public:
	graphic::cRenderTarget m_renderTarget;
	phys::cPhysicsEngine m_physics;
	phys::cPhysicsSync *m_physSync;
	graphic::cGridLine m_grid;

	int m_boxId;
	physx::PxArticulationReducedCoordinate* m_articulation;
	physx::PxArticulationJointReducedCoordinate* m_driveJoint;
	phys::cArticulation m_art;
	graphic::cCube m_box;

	bool m_showGrid;

	// MouseMove Variable
	POINT m_viewPos;
	sRectf m_viewRect; // detect mouse event area
	POINT m_mousePos; // window 2d mouse pos
	Vector3 m_mousePickPos; // mouse cursor pos in ground picking
	bool m_mouseDown[3]; // Left, Right, Middle
	float m_rotateLen;

	// physx simulation
	bool m_isSimulation; // start physx simulation?
	vector<Vector3> m_jointPoss;
};
