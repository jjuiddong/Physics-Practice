//
// 2020-01-16, jjuiddong
// Information View
//
#pragma once


class cInformationView : public framework::cDockWindow
{
public:
	cInformationView(const StrId &name);
	virtual ~cInformationView();

	virtual void OnUpdate(const float deltaSeconds) override;
	virtual void OnRender(const float deltaSeconds) override;


protected:
	void RenderFixedJoint();
	void RenderSphericalJoint();
	void RenderRevoluteJoint();
	void RenderPrismaticJoint();
	void RenderDistanceJoint();
	void RenderD6Joint();


public:
};
