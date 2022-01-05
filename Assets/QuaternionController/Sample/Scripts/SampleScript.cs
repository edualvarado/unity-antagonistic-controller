// <copyright file="SampleScript.cs" company="VacuumBreather">
//      Copyright © 2015 VacuumBreather. All rights reserved.
// </copyright>

namespace VacuumBreather
{
    #region Using Directives

    using UnityEngine;

    #endregion

    public class SampleScript : MonoBehaviour
    {
        #region Fields

        public Transform TargetOne;
        public Transform TargetTwo;
        public Transform TargetThree;
        public ControlledObject ControlledObject;

        private Vector3 _cameraPosition;

        #endregion

        #region Instance Methods

        private void Awake()
        {
            this.ControlledObject.GetComponent<Rigidbody>().centerOfMass = Vector3.zero;
            this._cameraPosition = Camera.main.transform.position;
        }

        private void OnGUI()
        {
            GUI.BeginGroup(new Rect(10, 10, 175, 450));

            // Make a background box
            GUI.Box(new Rect(10, 10, 150, 400), "Choose Action");

            if (GUI.Button(new Rect(20, 40, 125, 20), "Reset"))
            {
                if (this.ControlledObject != null)
                {
                    this.ControlledObject.transform.position = Vector3.zero;
                    this.ControlledObject.transform.rotation = Quaternion.Euler(0, 180, 0);
                    this.ControlledObject.DesiredOrientation = this.ControlledObject.transform.rotation;
                    this.ControlledObject.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
                    Camera.main.transform.position = this._cameraPosition;
                }
            }

            if (GUI.Button(new Rect(20, 70, 125, 20), "Match Target 1"))
            {
                if (this.ControlledObject != null)
                {
                    this.ControlledObject.DesiredOrientation = this.TargetOne.rotation;
                }
            }

            if (GUI.Button(new Rect(20, 100, 125, 20), "Match Target 2"))
            {
                if (this.ControlledObject != null)
                {
                    this.ControlledObject.DesiredOrientation = this.TargetTwo.rotation;
                }
            }

            if (GUI.Button(new Rect(20, 130, 125, 20), "Match Target 3"))
            {
                if (this.ControlledObject != null)
                {
                    this.ControlledObject.DesiredOrientation = this.TargetThree.rotation;
                }
            }

            if (GUI.Button(new Rect(20, 160, 125, 20), "Look at Target 1"))
            {
                if (this.ControlledObject != null)
                {
                    this.ControlledObject.DesiredOrientation = Quaternion.LookRotation(this.TargetOne.position, Vector3.up);
                }
            }

            if (GUI.Button(new Rect(20, 190, 125, 20), "Look at Target 2"))
            {
                if (this.ControlledObject != null)
                {
                    this.ControlledObject.DesiredOrientation = Quaternion.LookRotation(this.TargetTwo.position, Vector3.up);
                }
            }

            if (GUI.Button(new Rect(20, 220, 125, 20), "Look at Target 3"))
            {
                if (this.ControlledObject != null)
                {
                    this.ControlledObject.DesiredOrientation = Quaternion.LookRotation(this.TargetThree.position, Vector3.up);
                }
            }

            GUIStyle centeredStyle = GUI.skin.GetStyle("Label");
            centeredStyle.alignment = TextAnchor.UpperCenter;

            GUI.Label(new Rect(20, 250, 125, 60), "Use scrollwheel to zoom camera.", centeredStyle);

            GUI.EndGroup();
        }

        private void Update()
        {
            Vector3 delta = Input.mouseScrollDelta.y * Vector3.forward;

            Vector3 localizedDelta = Camera.main.transform.TransformDirection(delta);

            Camera.main.transform.position += localizedDelta;
        }

        #endregion
    }
}