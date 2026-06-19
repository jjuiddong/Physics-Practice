//
// 2026-06-05, jjuiddong
// urdf2 format script parsing class
//	- urdf2: no xml version urdf format
//	- seperate from robot ik, cUrdf2
//
// urdf2 hierarchy structure
// root link
//		- child visual (geometry)
//		- child collision (geometry)
//		- child joint
//			- child link
//				- child visual (geometry)
//				- child collision (geometry)
//				- child joint 
//					- ...
//
#pragma once


// robot inverse kinematics library
namespace rik
{

	//----------------------------------------------------------------------------------
	// urdf base class
	class cNode2
	{
	public:
		cNode2(const string& name, const int nodeType) : m_name(name), m_nodeType(nodeType)
			, m_parent(nullptr), m_color(graphic::cColor(1.f, 1.f, 1.f, 0.7f)) {}
		virtual ~cNode2();
		bool IsEndEffector() {
			return m_parent && (m_parent->m_children.size() == 1);
		}
		Quaternion EulerToQuat(const Vector3& euler);
		void Clear();
	public:
		int m_nodeType; // node type, 0:node, 1:link, 2:joint, 3:visual
		string m_name;
		cNode2* m_parent;
		Transform m_transform; // local pose
		Vector3 m_euler; // origin rotation, (roll/pitch/yaw, degree)
		graphic::cColor m_color;
		vector<cNode2*> m_children;
	};

	//----------------------------------------------------------------------------------
	// urdf link
	class cLink2 : public cNode2
	{
	public:
		cLink2(const string& name = "link") : cNode2(name, 1) {}
		virtual ~cLink2() {}
	};

	//----------------------------------------------------------------------------------
	// urdf visual(geometry)
	class cVisual2 : public cNode2
	{
	public:
		cVisual2(const string& name = "visual") : cNode2(name, 3)
			, m_geoType(0), m_radius(0.1f), m_height(0.1f) {}
		virtual ~cVisual2() {}
	public:
		int m_geoType; // 0:box, 1:sphere, 2:cylinder
		Vector3 m_dimension; // local space dimension
		float m_radius; // sphere, cylinder
		float m_height; // cylinder
	};

	//----------------------------------------------------------------------------------
	// urdf joint
	class cJoint2 : public cNode2
	{
	public:
		cJoint2(const string& name = "joint") : cNode2(name, 2), m_type(4), m_val(0.f)
			, m_upper(FLT_MAX), m_lower(-FLT_MAX)
			, m_mimicJointIdx(-1), m_isMimicJoint(false), m_mimicMultiplier(1.f)
			, m_mimicOffset(0.f), m_pidx(-1), m_cidx(-1) {}
		virtual ~cJoint2() {}
	public:
		int m_type; // joint type, 0:revolute, 1:prismatic, 3:universal, 4:fixed
		int m_pidx; // parent joint index
		int m_cidx; // child joint index
		string m_parentLink; // parent link name
		string m_childLink; // child link name
		Vector3 m_axis; // revolute axis direction (local space)
		float m_upper; // joint limit upper range (radian)
		float m_lower; // joint limit lower range (radian)
		float m_val; // joint rotation value (radian)

		// mimic joint variable
		bool m_isMimicJoint; // is mimic joint?
		int m_mimicJointIdx; // mimic joint index, default:-1 (dest joint index)
		string m_mimicJoint; // mimic joint name (source joint name)
		float m_mimicMultiplier; // mimic joint multiplier
		float m_mimicOffset; // mimic joint offset
		Matrix44 m_tm; // joint world transform (internal use)
	};


	//----------------------------------------------------------------------------------
	// urdf
	class cUrdf3
	{
	public:
		cUrdf3(const string& name = "");
		virtual ~cUrdf3();
		bool Open(const string& fileName);
		bool OpenFromString(const string& code);
		cNode2* Traverse(std::function<bool(cNode2* node, const Matrix44& tm)> fn
			, const vector<float>& values = {});
		virtual void Clear();

	protected:
		bool UpdateJointAttr();
		bool ConvertSpace();

	public:
		string m_name; // filename
		cNode2* m_root; // urdf tree root
		vector<cJoint2*> m_joints; // all joint, reference
		vector<Vector3> m_jointsPos; // for debugging
	};

}
