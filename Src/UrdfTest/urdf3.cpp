
#include "stdafx.h"
#include "urdf3.h"

using namespace rik;


//----------------------------------------------------------------------------------
// cNode2
cNode2::~cNode2()
{
	Clear();
}
void cNode2::Clear()
{
	for (auto& p : m_children)
		delete p;
	m_children.clear();
}

// euler (roll/pitch/yaw) -> quaternion
//		roll:x-axis, pitch:z-axis, yaw:y-axis
// euler: degree angle
Quaternion cNode2::EulerToQuat(const Vector3& euler)
{
	if (euler.IsEmpty())
		return Quaternion(); // zero rotation
	const Quaternion xRot(Vector3(1, 0, 0), ANGLE2RAD(euler.x)); // roll
	const Quaternion yRot(Vector3(0, 1, 0), ANGLE2RAD(euler.z)); // yaw
	const Quaternion zRot(Vector3(0, 0, 1), ANGLE2RAD(euler.y)); // pitch
	return xRot * zRot * yRot;
}


//----------------------------------------------------------------------------------
// cUrdf3
rik::cUrdf3::cUrdf3(
	const string& name //=""
)
	: m_root(nullptr)
	, m_name(name)
{
}

rik::cUrdf3::~cUrdf3()
{
	Clear();
}


// open urdf2 file
bool rik::cUrdf3::Open(const string& fileName)
{
	string code;
	common::cFile file;
	if (!file.ReadFile2String(fileName, code))
		return false;
	const bool res = OpenFromString(code);
	if (res)
		m_name = fileName;
	return res;
}


// open from urdf2 format string
// code: urdf2 format string
bool rik::cUrdf3::OpenFromString(const string& code)
{
	// urdf2 file sample
	//link
	//	name base
	//		origin_xyz 0 0 0
	//		visual
	//			geometry box
	//			dimension 1 0.1 1
	//			origin_xyz 0 0.05 0
	//	link
	//		name link1
	//		origin_xyz 0 0 0
	//		visual
	//			geometry box
	//			dimension 0.5 0.4 0.5
	//			origin_xyz 0 0.2 0
	//joint
	//	name joint1
	//	type revolute
	//	parent base
	//	child arm1
	//	origin_rpy 0 0 0
	//	origin_xyz 0 0 0
	//	axis_xyz 0 1 0
	// joint
	//	name joint2
	//	type revolute
	//	parent arm1
	//	child arm2
	//	origin_rpy 0 0 0
	//	origin_xyz 0 0 0
	//	axis_xyz 0 1 0

	// parse urdf2 file rule
	vector<cSimpleData2::sRule> rules;
	rules.push_back({ 0, "link", 1, 0 });
	rules.push_back({ 1, "link", 1, 0 });
	rules.push_back({ 2, "link", 1, 0 });
	rules.push_back({ 1, "visual", 3, 1 });
	rules.push_back({ 3, "visual", 3, 1 });
	rules.push_back({ 3, "link", 1, 0 });
	rules.push_back({ 0, "joint", 2, 0 });
	rules.push_back({ 1, "joint", 2, 0 });
	rules.push_back({ 2, "joint", 2, 0 });
	rules.push_back({ 3, "joint", 2, 0 });

	cSimpleData2 sdata(rules);
	stringstream ss(code);
	sdata.Read(ss);

	Clear();

	vector<cLink2*> links;
	vector<cJoint2*> joints;

	for (auto& data : sdata.m_root->children)
	{
		if (data->name == "link")
		{
			cLink2* link = new cLink2();
			link->m_name = sdata.Get<string>(data, "name", "");
			link->m_transform.pos = sdata.Get<Vector3>(data, "origin_xyz", Vector3(0, 0, 0));

			for (auto& child : data->children)
			{
				if (child->name == "visual")
				{
					cVisual2* vis = new cVisual2();

					const string geoType = sdata.Get<string>(child, "geometry", "");
					if (geoType == "box")
						vis->m_geoType = 0;
					else if (geoType == "sphere")
						vis->m_geoType = 1;
					else if (geoType == "cylinder")
						vis->m_geoType = 2;
					else if (geoType == "mesh")
						vis->m_geoType = 0;
					vis->m_transform.pos = sdata.Get<Vector3>(child, "origin_xyz", Vector3(0, 0, 0));
					vis->m_radius = sdata.Get<float>(child, "radius", 0.f);
					vis->m_height = max(sdata.Get<float>(child, "length", 0.f)
						, sdata.Get<float>(child, "height", 0.f));
					vis->m_euler = sdata.Get<Vector3>(child, "origin_rpy", Vector3(0, 0, 0));
					vis->m_transform.rot = vis->EulerToQuat(vis->m_euler);
					vis->m_dimension = sdata.Get<Vector3>(child, "dimension", Vector3(1, 1, 1));

					link->m_children.push_back(vis);
				}
			}

			links.push_back(link);
		}
		else if (data->name == "joint")
		{
			cJoint2* joint = new cJoint2();
			joint->m_name = sdata.Get<string>(data, "name", "");
			const string type = sdata.Get<string>(data, "type", "");
			if (type == "revolute")
				joint->m_type = 0;
			else if (type == "linear")
				joint->m_type = 2;
			else if (type == "universal")
				joint->m_type = 3;
			else if (type == "continuous")
				joint->m_type = 0; // same as revolute
			else
				joint->m_type = 4; // fixed

			joint->m_parentLink = sdata.Get<string>(data, "parent", "");
			joint->m_childLink = sdata.Get<string>(data, "child", "");
			joint->m_mimicJoint = sdata.Get<string>(data, "mimic", "");
			joint->m_transform.pos = sdata.Get<Vector3>(data, "origin_xyz", Vector3(0, 0, 0));
			joint->m_euler = sdata.Get<Vector3>(data, "origin_rpy", Vector3(0, 0, 0));
			joint->m_transform.rot = joint->EulerToQuat(joint->m_euler);
			joint->m_axis = sdata.Get<Vector3>(data, "axis_xyz", Vector3(1, 0, 0));
			joint->m_axis.Normalize();
			const Vector2 range = sdata.Get<Vector2>(data, "limit", Vector2(-FLT_MAX, FLT_MAX));
			joint->m_upper = max(range.x, range.y);
			joint->m_lower = min(range.x, range.y);
			if (FLT_MAX != joint->m_upper)
				joint->m_upper = ANGLE2RAD(joint->m_upper);
			if (-FLT_MAX != joint->m_lower)
				joint->m_lower = ANGLE2RAD(joint->m_lower);
			joint->m_mimicMultiplier = sdata.Get<float>(data, "multiplier", 1.f);
			joint->m_mimicOffset = sdata.Get<float>(data, "offset", 0.f);

			joints.push_back(joint);
		}
	}

	// make hierarchy
	// find root link
	vector<uint> parentCnt(links.size(), 0);
	vector<uint> childCnt(links.size(), 0);
	for (auto& j : joints)
	{
		{
			auto it = find_if(links.begin(), links.end()
				, [&](const auto& a) { return a->m_name == j->m_parentLink; });
			if (links.end() == it)
				continue; // ignore
			const int idx = (int)(it - links.begin());
			++parentCnt[idx];
		}
		{
			auto it = find_if(links.begin(), links.end()
				, [&](const auto& a) { return a->m_name == j->m_childLink; });
			if (links.end() == it)
				continue; // ignore
			const int idx = (int)(it - links.begin());
			++childCnt[idx];
		}
	}
	cLink2* root = nullptr;
	for (uint i = 0; i < links.size(); ++i)
	{
		if ((parentCnt[i] > 0) && (0 == childCnt[i]))
		{
			root = links[i];
			break;
		}
	}
	if (!root)
	{
		for (auto& p : links)
		{
			for (auto& c : p->m_children)
				delete c;
			delete p;
		}
		for (auto& p : joints)
			delete p;
		return false;
	}
	//~find root

	for (auto& j : joints)
	{
		cLink2* parent = nullptr, * child = nullptr;
		{
			auto it = find_if(links.begin(), links.end()
				, [&](const auto& a) { return a->m_name == j->m_parentLink; });
			if (links.end() == it)
				continue; // ignore
			parent = *it;
		}
		{
			auto it = find_if(links.begin(), links.end()
				, [&](const auto& a) { return a->m_name == j->m_childLink; });
			if (links.end() == it)
				continue; // ignore
			child = *it;
		}
		if (!parent || !child)
			continue; // error, ignore

		parent->m_children.push_back(j);
		j->m_parent = parent;
		j->m_children.push_back(child);
		child->m_parent = j;
	}

	// update parent,child 
	struct sNode {
		cNode2* parent;
		cNode2* cur;
	};
	queue<sNode> q;
	q.push({ nullptr, root });
	while (!q.empty())
	{
		sNode node = q.front(); q.pop();
		node.cur->m_parent = node.parent;
		for (auto& child : node.cur->m_children)
			q.push({ node.cur, child });
	}
	//~

	m_root = root;
	UpdateJointAttr(); // must call after m_root update

	return true;
}


// update joint distance
bool rik::cUrdf3::UpdateJointAttr()
{
	struct sJointInfo
	{
		cJoint2* joint;
		Matrix44 wtm; // world tm
	};
	struct sLinkInfo
	{
		cLink2* link;
		Matrix44 wtm; // world tm
	};

	// update joint info (type, direction, distance, etc..)
	// urdf hierarchy
	// joints: root joint - child joint - child joint ...
	// links: base link - child link - child link ...
	vector<sJointInfo> joints;
	vector<sLinkInfo> links;
	Traverse([&](cNode2* node, const Matrix44& tm) {
		if (cJoint2* joint = dynamic_cast<cJoint2*>(node))
			joints.push_back({ joint, tm });
		else if (cLink2* link = dynamic_cast<cLink2*>(node))
			links.push_back({ link, tm });
		return false; // continuous loop
		});

	if (joints.empty())
		return false; // nothing to do
	if (links.size() != (joints.size() + 1))
		return false; // error return

	// update joint parent, child joint index
	for (uint i = 0; i < joints.size(); ++i)
	{
		cJoint2* j0 = joints[i].joint;
		for (uint k = 0; k < joints.size(); ++k)
		{
			if (i != k)
			{
				// tricky code: continue update index because last joint is real link joint
				if (j0->m_parentLink == joints[k].joint->m_childLink)
					j0->m_pidx = k;
				if (j0->m_childLink == joints[k].joint->m_parentLink)
					j0->m_cidx = k;
			}
		}
	}
	//~

	// update mimic joint
	for (uint i = 0; i < joints.size(); ++i)
	{
		if (joints[i].joint->m_mimicJoint.empty())
			continue;
		for (uint k = 0; k < joints.size(); ++k)
		{
			if ((i != k)
				&& (joints[i].joint->m_mimicJoint == joints[k].joint->m_name))
			{
				joints[k].joint->m_mimicJointIdx = i;
				joints[i].joint->m_isMimicJoint = true;
			}
		}
	}
	//~

	m_joints.clear();
	for (auto& info : joints)
		m_joints.push_back(info.joint);
	m_jointsPos.resize(joints.size());
	return true;
}


// traverse all node 
// fn: traverse function, return false, stop traverse
// values: joint values
rik::cNode2* rik::cUrdf3::Traverse(std::function<bool(cNode2* node, const Matrix44& tm)> fn
	, const vector<float>& values //= {}
)
{
	RETV(!m_root, nullptr);
	struct sChunk
	{
		cNode2* node;
		Matrix44 mat;
	};
	queue<sChunk> q;
	q.push({ m_root, Matrix44::Identity });
	uint jointIdx = 0;
	while (!q.empty())
	{
		sChunk chunk = q.front(); q.pop();
		cNode2* node = chunk.node;
		Matrix44 mat = chunk.mat;
		Matrix44 tm;
		if (cJoint2* joint = dynamic_cast<cJoint2*>(node))
		{
			Transform tfm = joint->m_transform;
			const float val = (values.size() > jointIdx) ?
				values[jointIdx] : joint->m_val;
			// revolute joint?
			if (0 == joint->m_type)
				tfm.rot = Quaternion(joint->m_axis, val) * tfm.rot;
			tm = tfm.GetMatrix() * mat;
			joint->m_tm = tm; // update joint world transform
			++jointIdx;
		}
		else
		{
			tm = node->m_transform.GetMatrix() * mat;
		}
		if (fn(node, tm))
			return node; // finish traverse
		for (auto& p : node->m_children)
			q.push({ p, tm });
	}
	return nullptr;
}


// clear
void rik::cUrdf3::Clear()
{
	m_joints.clear();
	SAFE_DELETE(m_root);
}
