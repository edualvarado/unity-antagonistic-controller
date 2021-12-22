using System;
using UnityEditor;
using UnityEngine;

namespace BzKovSoft.RagdollHelper.Editor
{
	public class RagdollProperties
	{
		public bool asTrigger;
		public bool isKinematic;
		public bool useGravity = true;
		public bool createTips = true;
		public float rigidDrag;
		public float rigidAngularDrag;
		public CollisionDetectionMode cdMode;

		internal void Draw()
		{
			cdMode = (CollisionDetectionMode)EditorGUILayout.EnumPopup("Collision detection:", cdMode);

			rigidDrag = EditorGUILayout.FloatField("Rigid Drag:", rigidDrag);

			rigidAngularDrag = EditorGUILayout.FloatField("Rigid Angular Drag:", rigidAngularDrag);

			asTrigger = EditorGUILayout.Toggle("Trigger colliders:", asTrigger);

			isKinematic = EditorGUILayout.Toggle("Is kinematic:", isKinematic);

			useGravity = EditorGUILayout.Toggle("Use gravity:", useGravity);
			createTips = EditorGUILayout.Toggle("Create tips:", createTips);
		}
	}
}