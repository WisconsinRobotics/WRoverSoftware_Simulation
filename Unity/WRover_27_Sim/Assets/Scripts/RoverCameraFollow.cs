using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoverCameraFollow : MonoBehaviour
{
    [Header("Target")]
    // Exposes the target Transform in the Unity Inspector for assignment
    [SerializeField] private Transform target;

    [Header("Offset & Distances")]
    [SerializeField] private Vector3 targetOffset = new Vector3(0f, 0.8f, 0f);
    [SerializeField] private float distance = 4.5f;
    [SerializeField] private float height = 2.0f;

    [Header("Smoothing Speeds")]
    [SerializeField] private float followDamping = 5.0f;
    [SerializeField] private float rotationDamping = 3.0f;

    private void LateUpdate()
    {
        if (!target) return;

        // Calculate target positions
        Vector3 targetCenter = target.position + targetOffset;
        // Align along X axis instead of Z as rover moves forward along X axis.
        Vector3 wantedPosition = target.position - (target.right * distance) + (Vector3.up * height);

        // Smoothly interpolate position
        transform.position = Vector3.Lerp(transform.position, wantedPosition, followDamping * Time.deltaTime);

        // Smoothly look toward target center
        Vector3 lookDirection = (targetCenter - transform.position).normalized;
        if (lookDirection != Vector3.zero)
        {
            Quaternion wantedRotation = Quaternion.LookRotation(lookDirection, Vector3.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, wantedRotation, rotationDamping * Time.deltaTime);
        }
    }

    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
    }
}
