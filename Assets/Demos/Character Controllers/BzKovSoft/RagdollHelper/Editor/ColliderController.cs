using System;
using UnityEditor;
using UnityEngine;

namespace BzKovSoft.RagdollHelper.Editor
{
	class ColliderController
	{
		public static void DrawControllers(BoneHelper boneHelper, Quaternion lastRotation, Transform transform, Vector3 pos)
		{
			Quaternion rotatorRotation = ColliderHelper.GetRotatorRotarion(transform);

			switch (Tools.current)
			{
				case Tool.Rotate:
					ProcessRotation(boneHelper, rotatorRotation, lastRotation, transform, pos);
					break;
				case Tool.Move:
					ProcessColliderMove(rotatorRotation, transform, pos);
					break;
				case Tool.Rect:
				case Tool.Scale:
					ProcessColliderScale(boneHelper, rotatorRotation, transform, pos);
					break;
			}
		}
		/// <summary>
		/// Rotate node's colider though controls
		/// </summary>
		static void ProcessRotation(BoneHelper boneHelper, Quaternion rotatorRotation, Quaternion lastRotation, Transform transform, Vector3 pos)
		{
			Quaternion newRotation;
			bool changed;

			if (Tools.pivotRotation == PivotRotation.Global)
			{
				Quaternion fromStart = rotatorRotation * Quaternion.Inverse(lastRotation);
				newRotation = Handles.RotationHandle(fromStart, pos);
				changed = fromStart != newRotation;
				newRotation = newRotation * lastRotation;
			}
			else
			{
				newRotation = Handles.RotationHandle(rotatorRotation, pos);
				changed = rotatorRotation != newRotation;
			}

			if (changed)
			{
				transform = ColliderHelper.GetRotatorTransform(transform);
				ColliderHelper.RotateCollider(transform, newRotation);
			}
		}

		/// <summary>
		/// Resize collider though controls
		/// </summary>
		/// <param name="transform">The node the collider is attached to</param>
		static void ProcessColliderMove(Quaternion rotatorRotation, Transform transform, Vector3 pos)
		{
			if (Tools.pivotRotation == PivotRotation.Global)
				rotatorRotation = Quaternion.identity;

			Vector3 newPosition = Handles.PositionHandle(pos, rotatorRotation);
			Vector3 translateBy = newPosition - pos;

			if (translateBy != Vector3.zero)
				ColliderHelper.SetColliderPosition(transform, newPosition);
		}

		/// <summary>
		/// Move collider though controls
		/// </summary>
		static void ProcessColliderScale(BoneHelper boneHelper, Quaternion rotatorRotation, Transform transform, Vector3 pos)
		{
			float size = HandleUtility.GetHandleSize(pos);
			var collider = ColliderHelper.GetCollider(transform);

			// process each collider type in its own way
			CapsuleCollider cCollider = collider as CapsuleCollider;
			BoxCollider bCollider = collider as BoxCollider;
			SphereCollider sCollider = collider as SphereCollider;

			if (cCollider != null)
			{
				// for capsule collider draw circle and two dot controllers
				Vector3 direction = DirectionIntToVector(cCollider.direction);

				var t = Quaternion.LookRotation(cCollider.transform.TransformDirection(direction));

				// method "Handles.ScaleValueHandle" multiplies size on 0.15f
				// so to send exact size to "Handles.CircleCap",
				// I needed to multiply size on 1f/0.15f
				// Then to get a size a little bigger (to 130%) than
				// collider (for nice looking purpose), I multiply size by 1.3f
				const float magicNumber = 1f / 0.15f * 1.3f;

				// draw radius controll

				float scale = (cCollider.transform.lossyScale.x + cCollider.transform.lossyScale.y + cCollider.transform.lossyScale.z) / 3f;
				float radius = Handles.ScaleValueHandle(cCollider.radius, pos, t, cCollider.radius * magicNumber * scale, Handles.CircleHandleCap, 0);
				bool radiusChanged = cCollider.radius != radius;

				Vector3 scaleHeightShift = cCollider.transform.TransformDirection(direction * cCollider.height / 2);

				// draw height controlls
				Vector3 heightControl1Pos = pos + scaleHeightShift;
				Vector3 heightControl2Pos = pos - scaleHeightShift;

				float height1 = Handles.ScaleValueHandle(cCollider.height, heightControl1Pos, t, size * 0.5f, Handles.DotHandleCap, 0);
				float height2 = Handles.ScaleValueHandle(cCollider.height, heightControl2Pos, t, size * 0.5f, Handles.DotHandleCap, 0);
				float newHeight = 0f;

				bool moved = false;
				bool firstCtrlSelected = false;
				if (height1 != cCollider.height)
				{
					moved = true;
					firstCtrlSelected = true;
					newHeight = height1;
				}
				else if (height2 != cCollider.height)
				{
					moved = true;
					newHeight = height2;
				}

				if (moved | radiusChanged)
				{
					Undo.RecordObject(cCollider, "Resize capsule collider");

					bool upperSelected = false;
					if (moved)
					{
						if (newHeight < 0.01f)
							newHeight = 0.01f;

						bool firstIsUpper = FirstIsUpper(cCollider.transform, heightControl1Pos, heightControl2Pos);
						upperSelected = firstIsUpper == firstCtrlSelected;

						cCollider.center += direction * (newHeight - cCollider.height) / 2 * (firstCtrlSelected ? 1 : -1);
						cCollider.height = newHeight;
					}
					if (radiusChanged)
						cCollider.radius = radius;

					// resize symmetric colliders too
					Transform symBone;
					if (boneHelper.SymmetricBones != null && boneHelper.SymmetricBones.TryGetValue(transform.name, out symBone))
					{
						var symCapsule = ColliderHelper.GetCollider(symBone) as CapsuleCollider;
						if (symCapsule == null)
							return;

						Undo.RecordObject(symCapsule, "Resize symetric capsule collider");

						if (moved)
						{
							Vector3 direction2 = DirectionIntToVector(symCapsule.direction);

							Vector3 scaleHeightShift2 = symCapsule.transform.TransformDirection(direction2 * symCapsule.height / 2);
							Vector3 pos2 = ColliderHelper.GetRotatorPosition(symCapsule.transform);

							Vector3 heightControl1Pos2 = pos2 + scaleHeightShift2;
							Vector3 heightControl2Pos2 = pos2 - scaleHeightShift2;

							bool firstIsUpper2 = FirstIsUpper(symCapsule.transform, heightControl1Pos2, heightControl2Pos2);

							symCapsule.center += direction2 * (newHeight - symCapsule.height) / 2
								* (upperSelected ? 1 : -1)
								* (firstIsUpper2 ? 1 : -1);

							symCapsule.height = cCollider.height;
						}
						if (radiusChanged)
							symCapsule.radius = cCollider.radius;
					}
				}
			}
			else if (bCollider != null)
			{
				// resize Box collider

				var newSize = Handles.ScaleHandle(bCollider.size, pos, rotatorRotation, size);
				if (bCollider.size != newSize)
				{
					Undo.RecordObject(bCollider, "Resize box collider");
					bCollider.size = newSize;
				}
			}
			else if (sCollider != null)
			{
				// resize Sphere collider
				var newRadius = Handles.RadiusHandle(rotatorRotation, pos, sCollider.radius, true);
				if (sCollider.radius != newRadius)
				{
					Undo.RecordObject(sCollider, "Resize sphere collider");
					sCollider.radius = newRadius;
				}
			}
			else
				throw new InvalidOperationException("Unsupported Collider type: " + collider.GetType().FullName);
		}

		/// <summary>
		/// Int (Physx spesific) direction to Vector3 direction
		/// </summary>
		static Vector3 DirectionIntToVector(int direction)
		{
			Vector3 v;
			switch (direction)
			{
				case 0:
					v = Vector3.right;
					break;
				case 1:
					v = Vector3.up;
					break;
				case 2:
					v = Vector3.forward;
					break;
				default:
					throw new InvalidOperationException();
			}
			return v;
		}

		private static bool FirstIsUpper(Transform transform, Vector3 heightControl1Pos, Vector3 heightControl2Pos)
		{
			if (transform.parent == null)
				return true;

			Vector3 currentPos = transform.position;
			Vector3 parentPos;
			do
			{
				transform = transform.parent;
				parentPos = transform.position;
			}
			while (parentPos == currentPos & transform.parent != null);

			if (parentPos == currentPos)
				return true;

			Vector3 limbDirection = currentPos - parentPos;

			limbDirection.Normalize();

			float d1 = Vector3.Dot(limbDirection, heightControl1Pos - parentPos);
			float d2 = Vector3.Dot(limbDirection, heightControl2Pos - parentPos);


			bool firstIsUpper = d1 < d2;
			return firstIsUpper;
		}

		public static Vector3 GetPos(Transform transform)
		{
			return ColliderHelper.GetRotatorPosition(transform);
		}
	}
}