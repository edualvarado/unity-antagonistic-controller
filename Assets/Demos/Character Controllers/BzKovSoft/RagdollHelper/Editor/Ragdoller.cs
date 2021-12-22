using UnityEngine;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace BzKovSoft.RagdollHelper.Editor
{
	/// <summary>
	/// Class responsible for regdoll and unregdoll character
	/// </summary>
	sealed class Ragdoller
	{
		const string _colliderNodeSufix = "_ColliderRotator";
		readonly bool _readyToGenerate;
		readonly Vector3 _playerDirection;
		readonly Transform _rootNode;

		readonly RagdollPartBox _pelvis;
		readonly RagdollPartCapsule _leftHip;
		readonly RagdollPartCapsule _leftKnee;
		readonly RagdollPartCapsule _rightHip;
		readonly RagdollPartCapsule _rightKnee;
		readonly RagdollPartCapsule _leftArm;
		readonly RagdollPartCapsule _leftElbow;
		readonly RagdollPartCapsule _rightArm;
		readonly RagdollPartCapsule _rightElbow;
		readonly RagdollPartBox _chest;
		readonly RagdollPartSphere _head;

		readonly RagdollPartBox _leftFoot;
		readonly RagdollPartBox _rightFoot;
		readonly RagdollPartBox _leftHand;
		readonly RagdollPartBox _rightHand;

		public Ragdoller(Transform player, Vector3 playerDirection)
		{
			_playerDirection = playerDirection;
			_readyToGenerate = false;

			// find Animator
			Animator animator = FindAnimator(player);
			if (animator == null)
				return;
			_rootNode = animator.transform;

			// specify all parts of ragdoll
			_pelvis = new RagdollPartBox(animator.GetBoneTransform(HumanBodyBones.Hips));
			_leftHip = new RagdollPartCapsule(animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg));
			_leftKnee = new RagdollPartCapsule(animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg));
			_rightHip = new RagdollPartCapsule(animator.GetBoneTransform(HumanBodyBones.RightUpperLeg));
			_rightKnee = new RagdollPartCapsule(animator.GetBoneTransform(HumanBodyBones.RightLowerLeg));
			_leftArm = new RagdollPartCapsule(animator.GetBoneTransform(HumanBodyBones.LeftUpperArm));
			_leftElbow = new RagdollPartCapsule(animator.GetBoneTransform(HumanBodyBones.LeftLowerArm));
			_rightArm = new RagdollPartCapsule(animator.GetBoneTransform(HumanBodyBones.RightUpperArm));
			_rightElbow = new RagdollPartCapsule(animator.GetBoneTransform(HumanBodyBones.RightLowerArm));
			_chest = new RagdollPartBox(animator.GetBoneTransform(HumanBodyBones.Chest));
			_head = new RagdollPartSphere(animator.GetBoneTransform(HumanBodyBones.Head));

			_leftFoot = new RagdollPartBox(animator.GetBoneTransform(HumanBodyBones.LeftFoot));
			_rightFoot = new RagdollPartBox(animator.GetBoneTransform(HumanBodyBones.RightFoot));
			_leftHand = new RagdollPartBox(animator.GetBoneTransform(HumanBodyBones.LeftHand));
			_rightHand = new RagdollPartBox(animator.GetBoneTransform(HumanBodyBones.RightHand));

			if (_chest.transform == null)
				_chest = new RagdollPartBox(animator.GetBoneTransform(HumanBodyBones.Spine));

			if (!CheckFields())
			{
				Debug.LogError("Not all nodes was found!");
				return;
			}

			_readyToGenerate = true;
		}
		/// <summary>
		/// Finds animator component in "player" and in parents till it find Animator component. Otherwise returns null
		/// </summary>
		static Animator FindAnimator(Transform player)
		{
			Animator animator;
			do
			{
				animator = player.GetComponent<Animator>();
				if (animator != null && animator.enabled)
					break;

				player = player.parent;
			}
			while (player != null);

			if (animator == null | player == null)
			{
				Debug.LogError("An Animator must be attached to find bones!");
				return null;
			}
			if (!animator.isHuman)
			{
				Debug.LogError("To auto detect bones, there are has to be humanoid Animator!");
				return null;
			}
			return animator;
		}
		/// <summary>
		/// Some checks before Applying ragdoll
		/// </summary>
		bool CheckFields()
		{
			if (_rootNode == null |
				_pelvis == null |
				_leftHip == null |
				_leftKnee == null |
				_rightHip == null |
				_rightKnee == null |
				_leftArm == null |
				_leftElbow == null |
				_rightArm == null |
				_rightElbow == null |
				_chest == null |
				_head == null)
				return false;

			return true;
		}


		/// <summary>
		/// Create all ragdoll's components and set their proterties
		/// </summary>
		public void ApplyRagdoll(float totalMass, RagdollProperties ragdollProperties)
		{
			if (!_readyToGenerate)
			{
				Debug.LogError("Initialization failed. Reinstance object!");
				return;
			}

			var weight = new WeightCalculator(totalMass, ragdollProperties.createTips);

			bool alreadyRagdolled = _pelvis.transform.gameObject.GetComponent<Rigidbody>() != null;

			AddComponentesTo(_pelvis,     ragdollProperties, weight.Pelvis, false);
			AddComponentesTo(_leftHip,    ragdollProperties, weight.Hip,    true);
			AddComponentesTo(_leftKnee,   ragdollProperties, weight.Knee,   true);
			AddComponentesTo(_rightHip,   ragdollProperties, weight.Hip,    true);
			AddComponentesTo(_rightKnee,  ragdollProperties, weight.Knee,   true);
			AddComponentesTo(_leftArm,    ragdollProperties, weight.Arm,    true);
			AddComponentesTo(_leftElbow,  ragdollProperties, weight.Elbow,  true);
			AddComponentesTo(_rightArm,   ragdollProperties, weight.Arm,    true);
			AddComponentesTo(_rightElbow, ragdollProperties, weight.Elbow,  true);
			AddComponentesTo(_chest,      ragdollProperties, weight.Chest,  true);
			AddComponentesTo(_head,       ragdollProperties, weight.Head,   true);

			if (ragdollProperties.createTips)
			{
				AddComponentesTo(_leftFoot,   ragdollProperties, weight.Foot,   true);
				AddComponentesTo(_rightFoot,  ragdollProperties, weight.Foot,   true);
				AddComponentesTo(_leftHand,   ragdollProperties, weight.Hand,   true);
				AddComponentesTo(_rightHand,  ragdollProperties, weight.Hand,   true);
			}

			if (alreadyRagdolled)
				return;

			// Pelvis
			Vector3 pelvisSize = new Vector3(0.32f, 0.31f, 0.3f);
			Vector3 pelvisCenter = new Vector3(00f, 0.06f, -0.01f);
			_pelvis.collider.size = Abs(_pelvis.transform.InverseTransformVector(pelvisSize));
			_pelvis.collider.center = _pelvis.transform.InverseTransformVector(pelvisCenter);

			ApplySide(true, ragdollProperties.createTips);
			ApplySide(false, ragdollProperties.createTips);

			// Chest collider
			Vector3 chestSize = new Vector3(0.34f, 0.34f, 0.28f);

			float y = (pelvisSize.y + chestSize.y) / 2f + pelvisCenter.y;
			y -= _chest.transform.position.y - _pelvis.transform.position.y;
			_chest.collider.size = Abs(_chest.transform.InverseTransformVector(chestSize));
			_chest.collider.center = _chest.transform.InverseTransformVector(new Vector3(0f, y, -0.03f));

			// Chest joint
			var chestJoint = _chest.joint;
			ConfigureJointParams(_chest, _pelvis.rigidbody, _rootNode.right, _rootNode.forward);
			ConfigureJointLimits(chestJoint, -45f, 20f, 20f, 20f);

			// head
			_head.collider.radius = 0.1f;
			_head.collider.center = _head.transform.InverseTransformVector(new Vector3(0f, 0.09f, 0.03f));
			var headJoint = _head.joint;
			ConfigureJointParams(_head, _chest.rigidbody, _rootNode.right, _rootNode.forward);
			ConfigureJointLimits(headJoint, -45f, 20f, 20f, 20f);
		}

		private Vector3 Abs(Vector3 v)
		{
			return new Vector3(
				Mathf.Abs(v.x),
				Mathf.Abs(v.y),
				Mathf.Abs(v.z)
				);
		}

		static void ConfigureJointParams(RagdollPartBase part, Rigidbody anchor, Vector3 axis, Vector3 swingAxis)
		{
			part.joint.connectedBody = anchor;
			part.joint.axis = part.transform.InverseTransformDirection(axis);
			part.joint.swingAxis = part.transform.InverseTransformDirection(swingAxis);
		}

		static void ConfigureJointLimits(CharacterJoint joint, float lowTwist, float highTwist, float swing1, float swing2)
		{
			if (lowTwist > highTwist)
				throw new ArgumentException("wrong limitation: lowTwist > highTwist");

			var twistLimitSpring = joint.twistLimitSpring;
			joint.twistLimitSpring = twistLimitSpring;

			var swingLimitSpring = joint.swingLimitSpring;
			joint.swingLimitSpring = swingLimitSpring;

			// configure limits
			var lowTwistLimit = joint.lowTwistLimit;
			lowTwistLimit.limit = lowTwist;
			joint.lowTwistLimit = lowTwistLimit;
			var highTwistLimit = joint.highTwistLimit;
			highTwistLimit.limit = highTwist;
			joint.highTwistLimit = highTwistLimit;

			var swing1Limit = joint.swing1Limit;
			swing1Limit.limit = swing1;
			joint.swing1Limit = swing1Limit;
			var swing2Limit = joint.swing2Limit;
			swing2Limit.limit = swing2;
			joint.swing2Limit = swing2Limit;
		}

		/// <summary>
		/// Configure one hand and one leg
		/// </summary>
		/// <param name="leftSide">If true, configuration apply to left hand and left leg, otherwise right hand and right leg</param>
		void ApplySide(bool leftSide, bool createTips)
		{
			RagdollPartCapsule hip = (leftSide ? _leftHip : _rightHip);
			RagdollPartCapsule knee = (leftSide ? _leftKnee : _rightKnee);
			RagdollPartBox foot = (leftSide ? _leftFoot : _rightFoot);
			
			RagdollPartCapsule arm = (leftSide ? _leftArm : _rightArm);
			RagdollPartCapsule elbow = (leftSide ? _leftElbow : _rightElbow);
			RagdollPartBox hand = (leftSide ? _leftHand : _rightHand);

			ConfigureRagdollForLimb(hip, knee, foot, createTips);
			ConfigureLegsJoints(hip, knee, foot, createTips);

			ConfigureRagdollForLimb(arm, elbow, hand, createTips);
			ConfigureHandJoints(arm, elbow, hand, leftSide, createTips);
		}
		/// <summary>
		/// Configer one of 4 body parts: right leg, left leg, right hand or left hand
		/// </summary>
		static void ConfigureRagdollForLimb(RagdollPartCapsule limbUpper, RagdollPartCapsule limbLower, RagdollPartBox tip, bool createTips)
		{
			float totalLength = limbUpper.transform.InverseTransformPoint(tip.transform.position).magnitude;

			// limbUpper
			CapsuleCollider upperCapsule = limbUpper.collider;
			var boneEndPos = limbUpper.transform.InverseTransformPoint(limbLower.transform.position);
			upperCapsule.direction = GetXyzDirection(limbLower.transform.localPosition);
			upperCapsule.radius = totalLength * 0.12f;
			upperCapsule.height = boneEndPos.magnitude;
			upperCapsule.center = Vector3.Scale(boneEndPos, Vector3.one * 0.5f);

			// limbLower
			CapsuleCollider endCapsule = limbLower.collider;
			boneEndPos = limbLower.transform.InverseTransformPoint(tip.transform.position);
			endCapsule.direction = GetXyzDirection(boneEndPos);
			endCapsule.radius = totalLength * 0.12f;
			endCapsule.height = boneEndPos.magnitude;
			endCapsule.center = Vector3.Scale(boneEndPos, Vector3.one * 0.5f);

			// tip
			if (createTips)
			{
				boneEndPos = GetLongestTransform(tip.transform).position;
				boneEndPos = tip.transform.InverseTransformPoint(boneEndPos);

				Vector3 tipDir = GetXyzDirectionV(boneEndPos);
				Vector3 tipSides = (tipDir - Vector3.one) * -1;
				Vector3 boxSize = tipDir * boneEndPos.magnitude * 1.3f + tipSides * totalLength * 0.2f;

				BoxCollider tipBox = tip.collider;
				tipBox.size = boxSize;

				float halfTipLength = boneEndPos.magnitude / 2f;
				tipBox.center = Vector3.Scale(boneEndPos.normalized, Vector3.one * halfTipLength);
			}
		}

		private static Transform GetLongestTransform(Transform limb)
		{
			float longestF = -1;
			Transform longestT = null;

			// find the farest object that attached to 'limb'
			foreach (Transform t in limb.GetComponentsInChildren<Transform>())
			{
				float length = (limb.position - t.position).sqrMagnitude;
				if (length > longestF)
				{
					longestF = length;
					longestT = t;
				}
			}

			return longestT;
		}

		static Vector3 GetXyzDirectionV(Vector3 node)
		{
			var d = GetXyzDirection(node);

			switch (d)
			{
				case 0: return Vector3.right;
				case 1: return Vector3.up;
				case 2: return Vector3.forward;
			}

			throw new InvalidOperationException();
		}
		
		/// <summary>
		/// Get the most appropriate direction in terms of PhysX (0,1,2 directions)
		/// </summary>
		static int GetXyzDirection(Vector3 node)
		{
			float x = Mathf.Abs(node.x);
			float y = Mathf.Abs(node.y);
			float z = Mathf.Abs(node.z);

			if (x > y & x > z)		// x is the bigest
				return 0;
			if (y > x & y > z)		// y is the bigest
				return 1;

			// z is the bigest
			return 2;
		}

		void ConfigureHandJoints(RagdollPartCapsule arm, RagdollPartCapsule elbow, RagdollPartBox hand, bool leftHand, bool createTips)
		{
			var dirUpper = elbow.transform.position - arm.transform.position;
			var dirLower = hand.transform.position - elbow.transform.position;
			var dirHand = GetLongestTransform(hand.transform).position - hand.transform.position; // TODO: need to find the most longest bone

			if (leftHand)
			{
				ConfigureJointLimits(arm.joint, -100f, 30f, 100f, 45f);
				ConfigureJointLimits(elbow.joint, -120f, 0f, 10f, 90f);
				if (createTips)
				{
					ConfigureJointLimits(hand.joint, -90f, 90f, 90f, 45f);
				}
				dirUpper = -dirUpper;
				dirLower = -dirLower;
				dirHand = -dirHand;
			}
			else
			{
				ConfigureJointLimits(arm.joint, -30f, 100f, 100f, 45f);
				ConfigureJointLimits(elbow.joint, 0f, 120f, 10f, 90f);
				if (createTips)
				{
					ConfigureJointLimits(hand.joint, -90f, 90f, 90f, 45f);
				}
			}

			var upU = Vector3.Cross(_playerDirection, dirUpper);
			var upL = Vector3.Cross(_playerDirection, dirLower);
			var upH = Vector3.Cross(_playerDirection, dirHand);
			ConfigureJointParams(arm, _chest.rigidbody, upU, _playerDirection);
			ConfigureJointParams(elbow, arm.rigidbody, upL, _playerDirection);
			if (createTips)
			{
				ConfigureJointParams(hand, elbow.rigidbody, upH, _playerDirection);
			}
		}

		void ConfigureLegsJoints(RagdollPartCapsule hip, RagdollPartCapsule knee, RagdollPartBox foot, bool createTips)
		{
			var hipJoint = hip.joint;
			var kneeJoint = knee.joint;
			var footJoint = foot.joint;

			ConfigureJointParams(hip, _pelvis.rigidbody, _rootNode.right, _rootNode.forward);
			ConfigureJointParams(knee, hip.rigidbody, _rootNode.right, _rootNode.forward);

			ConfigureJointLimits(hipJoint, -10f, 120f, 90f, 20f);
			ConfigureJointLimits(kneeJoint, -120f, 0f, 10f, 20f);

			if (createTips)
			{
				ConfigureJointParams(foot, knee.rigidbody, _rootNode.right, _rootNode.forward);
				ConfigureJointLimits(footJoint, -70f, 70f, 45f, 20f);
			}
		}

		static void AddComponentesTo(RagdollPartBox part, RagdollProperties ragdollProperties, float mass, bool addJoint)
		{
			AddComponentesToBase(part, ragdollProperties, mass, addJoint);
			GameObject go = part.transform.gameObject;

			part.collider = GetCollider<BoxCollider>(go.transform);
			if (part.collider == null)
				part.collider = go.AddComponent<BoxCollider>();
			part.collider.isTrigger = ragdollProperties.asTrigger;
		}

		static void AddComponentesTo(RagdollPartCapsule part, RagdollProperties ragdollProperties, float mass, bool addJoint)
		{
			AddComponentesToBase(part, ragdollProperties, mass, addJoint);
			GameObject go = part.transform.gameObject;

			part.collider = GetCollider<CapsuleCollider>(go.transform);
			if (part.collider == null)
				part.collider = go.AddComponent<CapsuleCollider>();
			part.collider.isTrigger = ragdollProperties.asTrigger;
		}

		static void AddComponentesTo(RagdollPartSphere part, RagdollProperties ragdollProperties, float mass, bool addJoint)
		{
			AddComponentesToBase(part, ragdollProperties, mass, addJoint);
			GameObject go = part.transform.gameObject;

			part.collider = GetCollider<SphereCollider>(go.transform);
			if (part.collider == null)
				part.collider = go.AddComponent<SphereCollider>();
			part.collider.isTrigger = ragdollProperties.asTrigger;
		}

		static void AddComponentesToBase(RagdollPartBase part, RagdollProperties ragdollProperties, float mass, bool addJoint)
		{
			GameObject go = part.transform.gameObject;

			part.rigidbody = go.GetComponent<Rigidbody>();
			if (part.rigidbody == null)
				part.rigidbody = go.AddComponent<Rigidbody>();
			part.rigidbody.mass = mass;
			part.rigidbody.drag = ragdollProperties.rigidDrag;
			part.rigidbody.angularDrag = ragdollProperties.rigidAngularDrag;
			part.rigidbody.collisionDetectionMode = ragdollProperties.cdMode;
			part.rigidbody.isKinematic = ragdollProperties.isKinematic;
			part.rigidbody.useGravity = ragdollProperties.useGravity;

			if (addJoint)
			{
				part.joint = go.GetComponent<CharacterJoint>();
				if (part.joint == null)
					part.joint = go.AddComponent<CharacterJoint>();

				part.joint.enablePreprocessing = false;
				part.joint.enableProjection = true;
			}
		}

		static T GetCollider<T>(Transform transform)
			where T : Collider
		{
			for (int i = 0; i < transform.childCount; ++i)
			{
				Transform child = transform.GetChild(i);

				if (child.name.EndsWith(_colliderNodeSufix))
				{
					transform = child;
					break;
				}
			}

			return transform.GetComponent<T>();
		}

		/// <summary>
		/// Remove all colliders, joints, and rigids
		/// </summary>
		public void ClearRagdoll()
		{
			foreach (var component in _pelvis.transform.GetComponentsInChildren<Collider>())
				GameObject.DestroyImmediate(component);
			foreach (var component in _pelvis.transform.GetComponentsInChildren<CharacterJoint>())
				GameObject.DestroyImmediate(component);
			foreach (var component in _pelvis.transform.GetComponentsInChildren<Rigidbody>())
				GameObject.DestroyImmediate(component);

			DeleteColliderNodes(_pelvis.transform);
		}
		/// <summary>
		/// Correct deleting collider with collider's separate nodes
		/// </summary>
		/// <param name="node"></param>
		private static void DeleteColliderNodes(Transform node)
		{
			for (int i = 0; i < node.childCount; ++i)
			{
				Transform child = node.GetChild(i);

				if (child.name.EndsWith(_colliderNodeSufix))
					GameObject.DestroyImmediate(child.gameObject);
				else
					DeleteColliderNodes(child);
			}
		}
	}
}