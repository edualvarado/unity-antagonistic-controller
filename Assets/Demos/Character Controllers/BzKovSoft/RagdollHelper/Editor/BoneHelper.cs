using System.Globalization;
using UnityEditor;
using UnityEngine;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BzKovSoft.RagdollHelper.Editor
{
	/// <summary>
	/// Bone fixer. Main class that draws panel and do a lot of logic
	/// </summary>
	public sealed class BoneHelper : EditorWindow
	{
		[MenuItem("Window/BzSoft/Ragdoll Helper")]
		private static void ShowWindow()
		{
			EditorWindow.GetWindow(typeof(BoneHelper), false, "Ragdoll helper");
		}

		public Dictionary<string, Transform> SymmetricBones { get; private set; }
		object initialized;

		// current selected collider
		int _curPointIndex = -1;

		RagdollController _ragdollController;
		GameObject _go;
		Animator _animator;
		Transform _leftKnee;
		Transform _rightKnee;
		Transform _pelvis;
		
		Transform[] _transforms;

		bool _humanoidSelected;
		PivotMode _lastPivotMode;
		PivotRotation _lastPivotRotation;
		SelectedMode _selectedMode = SelectedMode.Ragdoll;
		SelectedMode _lastSelectedMode; // to detect, if mode changed from last frame
		readonly string[] _dropDownListOptions = {
			"Ragdoll",
			"Transform Colliders",
			"Transform Joints",
			"Rigid CenterOfMass",
		};
		enum SelectedMode
		{
			Ragdoll,
			Collider,
			Joints,
			CenterOfMass,
		}

		void OnSelectionChange()
		{
			_selectedMode = SelectedMode.Ragdoll;

			if (!GetTarget())
			{
				Repaint();
				return;
			}

			_ragdollController = new RagdollController(_go, () => GetPlayerDirection());

			Tools.hidden = false;

			_selectedMode = SelectedMode.Ragdoll;
			if (_humanoidSelected)
			{
				SymmetricBones = FindSymmetricBones(_animator);
				_leftKnee = _animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
				_rightKnee = _animator.GetBoneTransform(HumanBodyBones.RightLowerLeg);
				_pelvis = _animator.GetBoneTransform(HumanBodyBones.Hips);
			}
			else
			{
				SymmetricBones = null;
				_leftKnee = null;
				_rightKnee = null;
				_pelvis = null;
			}
			Repaint();
		}

		bool GetTarget()
		{
			_humanoidSelected = false;

			_go = Selection.activeGameObject;
			if (_go == null ||
				EditorUtility.IsPersistent(_go)) // if it is selected as asset, skip it
				return false;

			_animator = _go.GetComponent<Animator>();
			_humanoidSelected = _animator != null && _animator.isHuman;

			return true;
		}

		/// <summary>
		/// Find symmetric bones. (e.g. for left arm, it finds right arm and for right leg it finds left leg)
		/// </summary>
		static Dictionary<string, Transform> FindSymmetricBones(Animator animator)
		{
			var symBones = new Dictionary<string, Transform>();

            // feet
            symBones.Add(animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg).name, animator.GetBoneTransform(HumanBodyBones.RightLowerLeg));
			symBones.Add(animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg).name, animator.GetBoneTransform(HumanBodyBones.RightUpperLeg));

			symBones.Add(animator.GetBoneTransform(HumanBodyBones.RightLowerLeg).name, animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg));
			symBones.Add(animator.GetBoneTransform(HumanBodyBones.RightUpperLeg).name, animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg));

			// hands
			symBones.Add(animator.GetBoneTransform(HumanBodyBones.LeftLowerArm).name, animator.GetBoneTransform(HumanBodyBones.RightLowerArm));
			symBones.Add(animator.GetBoneTransform(HumanBodyBones.LeftUpperArm).name, animator.GetBoneTransform(HumanBodyBones.RightUpperArm));

			symBones.Add(animator.GetBoneTransform(HumanBodyBones.RightLowerArm).name, animator.GetBoneTransform(HumanBodyBones.LeftLowerArm));
			symBones.Add(animator.GetBoneTransform(HumanBodyBones.RightUpperArm).name, animator.GetBoneTransform(HumanBodyBones.LeftUpperArm));

			return symBones;
		}
		/// <summary>
		/// Find all "CapsuleCollider", "BoxCollider" and "SphereCollider" colliders
		/// </summary>
		void FindColliders()
		{
			if (_go == null)
			{
				_transforms = new Transform[0];
				return;
			}

			if (_selectedMode == SelectedMode.Collider)
			{
				var colliders = _go.GetComponentsInChildren<Collider>();

				_transforms = new Transform[colliders.Length];
				for (int i = 0; i < colliders.Length; ++i)
				{
					Transform transform = colliders[i].transform;
					if (transform.name.EndsWith(ColliderHelper.ColliderRotatorNodeSufix, false, CultureInfo.InvariantCulture))
					{
						transform = transform.parent;
					}
					_transforms[i] = transform;
				}
			}
			else if (_selectedMode == SelectedMode.Joints)
			{
				var joints = _go.GetComponentsInChildren<Joint>();

				_transforms = new Transform[joints.Length];
				for (int i = 0; i < joints.Length; ++i)
				{
					Transform transform = joints[i].transform;
					_transforms[i] = transform;
				}
			}
			else if (_selectedMode == SelectedMode.CenterOfMass)
			{
				var rigids = _go.GetComponentsInChildren<Rigidbody>();

				_transforms = new Transform[rigids.Length];
				for (int i = 0; i < rigids.Length; ++i)
				{
					Transform transform = rigids[i].transform;
					_transforms[i] = transform;
				}
			}
		}

		void OnEnable()
		{
			SceneView.onSceneGUIDelegate += OnSceneGUI;
		}
		void OnDisable()
		{
			SceneView.onSceneGUIDelegate -= OnSceneGUI;
			Tools.hidden = false;
		}

		void OnSceneGUI(SceneView sceneView)
		{
			CheckSelectedMode();

			if (_humanoidSelected)
				DrawPlayerDirection();

			if (_selectedMode == SelectedMode.Collider |
				_selectedMode == SelectedMode.Joints |
				_selectedMode == SelectedMode.CenterOfMass)
			{
				DrawControls();
			}
		}

		void OnGUI()
		{
			if (initialized == null)
			{
				OnSelectionChange();

				if (_go == null)
					return;

				initialized = new object();
			}

			CheckSelectedMode();

			DrawPanel();
		}

		/// <summary>
		/// Method intended to be invoked before Drawing GUI
		/// </summary>
		void CheckSelectedMode()
		{
			bool selectionChanged =
				_lastSelectedMode != _selectedMode |
				_lastPivotMode != Tools.pivotMode |
				_lastPivotRotation != Tools.pivotRotation;

			// if selected item was changed, research colliders
			if (selectionChanged)
			{
				Tools.hidden = _selectedMode != SelectedMode.Ragdoll;

				_lastSelectedMode = _selectedMode;
				_lastPivotMode = Tools.pivotMode;
				_lastPivotRotation = Tools.pivotRotation;
				FindColliders();
				SceneView.RepaintAll();
			}
		}
		/// <summary>
		/// Draw arrow on the screen, forwarded to front of the character
		/// </summary>
		void DrawPlayerDirection()
		{
			float size = HandleUtility.GetHandleSize(_go.transform.position);
			var playerDirection = GetPlayerDirection();
			Color backupColor = Handles.color;
			Handles.color = Color.yellow;
			Handles.ArrowHandleCap(1, _go.transform.position, Quaternion.LookRotation(playerDirection, Vector3.up), size, EventType.Repaint);
			Handles.color = backupColor;
		}
		
		Quaternion _lastRotation;
		bool _buttonPressed;

		/// <summary>
		/// Draws controls of selected mode.
		/// </summary>
		void DrawControls()
		{
			for (int i = 0; i < _transforms.Length; i++)
			{
				Transform transform = _transforms[i];

				if (transform == null)
					continue;

				Vector3 pos = Vector3.zero;

				switch (_selectedMode)
				{
					case SelectedMode.Collider:
						pos = ColliderController.GetPos(transform);
						break;
					case SelectedMode.Joints:
						pos = JointController.GetPos(transform);
						break;
					case SelectedMode.CenterOfMass:
						pos = RigidController.GetPos(transform);
						break;
				}

				float size = HandleUtility.GetHandleSize(pos) / 6f;

				if (Handles.Button(pos, Quaternion.identity, size, size, Handles.SphereHandleCap))
				{
					_curPointIndex = i;
				
					Quaternion rotatorRotation2 = ColliderHelper.GetRotatorRotarion(transform);
				
					if (!_buttonPressed)
					{
						_lastRotation = rotatorRotation2;
						_buttonPressed = true;
					}
				}
				else
					_buttonPressed = false;

				if (_curPointIndex != i)
					continue;

				// if current point controll was selected
				// draw other controls over it

				switch (_selectedMode)
				{
					case SelectedMode.Collider:
						ColliderController.DrawControllers(this, _lastRotation, transform, pos);
						break;
					case SelectedMode.Joints:
						JointController.DrawControllers(transform);
						break;
					case SelectedMode.CenterOfMass:
						RigidController.DrawControllers(transform);
						break;
				}
			}
		}

		/// <summary>
		/// Determines and return character's face direction
		/// </summary>
		public Vector3 GetPlayerDirection()
		{
			Vector3 leftKnee = _leftKnee.transform.position - _pelvis.transform.position;
			Vector3 rightKnee = _rightKnee.transform.position - _pelvis.transform.position;

			return Vector3.Cross(leftKnee, rightKnee).normalized;
		}
		/// <summary>
		/// Draw application form panel
		/// </summary>
		void DrawPanel()
		{
			// set the form more transparent if "NoAction" is selected
			// set size of form for each selected mode

			// draw panel
			Handles.BeginGUI();

			// draw list of modes to select
			GUIStyle style;
			style = new GUIStyle(GUI.skin.label);
			style.normal.textColor = Color.blue;
			style.alignment = TextAnchor.UpperCenter;

			GUILayout.Label(_humanoidSelected ? "Humanoid selected" : "Simple object selected", style);
			int selectedMode = (int)_selectedMode;
			selectedMode = GUILayout.SelectionGrid(selectedMode, _dropDownListOptions, 1, EditorStyles.radioButton);
			_selectedMode = (SelectedMode)selectedMode;

			// for Ragdoll mode draw additional controlls
			if (_selectedMode == SelectedMode.Ragdoll)
				_ragdollController.DrawRagdollPanel(_humanoidSelected);

			Handles.EndGUI();
		}

	}
}