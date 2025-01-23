//
// 2025-01-22, jjuiddong
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
	void InitRobot1();
	void InitRobot2();

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

	phys::cArticulation m_art;
	physx::PxArticulationJointReducedCoordinate* m_driveJoint;
	physx::PxArticulationJointReducedCoordinate* m_motorJoint1;
	physx::PxArticulationJointReducedCoordinate* m_motorJoint2;
	physx::PxArticulationJointReducedCoordinate* m_motorJoint3;
	physx::PxArticulationJointReducedCoordinate* m_motorJoint4;
	graphic::cCube m_box;
	graphic::cCylinder m_cylinder;
	graphic::cSphere m_sphere;

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
