using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Require component to have colliders, if the sphere does not have colliders, the script won't work
[RequireComponent(typeof(Collider))]

public class MouseFollow : MonoBehaviour
{
    private Camera camera;
    private float cameraDZ;

    void Start()
    {
        // Main camera object
        camera = Camera.main;
        // Distance of the sphere from screen view in Z axis
        cameraDZ = camera.WorldToScreenPoint(transform.position).z;
    }

    // Function that returns a Vector3 containing the position of the mouse in the 3D space
    Vector3 TransformMousePosition()
    {
        // Z axis added to mouse position
        Vector3 MousePosition = new Vector3(
            Input.mousePosition.x,
            Input.mousePosition.y,
            cameraDZ
        );

        // Screen mouse point converted to world point
        return camera.ScreenToWorldPoint(MousePosition); 
    }

    // Unity's built-in function to trigger mouse dragging and dropping
    void OnMouseDrag()
    {
        // Get mouse position in the 3D space
        Vector3 MouseWorldPosition = TransformMousePosition();

        // Change sphere's position
        transform.position = MouseWorldPosition;
    }
}