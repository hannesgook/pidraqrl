// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL
// https://github.com/hannesgook/pidraqrl

using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class BLETargetAngleController : MonoBehaviour
{
    private BLEIMUReceiver receiver;
    private Rigidbody rb;

    [Header("Visual Smoothing")]
    [Tooltip("How quickly the visual rotation tracks the target angle. Lower = smoother, higher = more responsive.")]
    [SerializeField, Range(1f, 50f)] private float trackingSpeed = 12f;

    [Header("Axis Locks")]
    [SerializeField] private bool enableRotationX = false;
    [SerializeField] private bool enableRotationY = false;
    [SerializeField] private bool enableRotationZ = true;

    [Header("Target Angle")]
    [SerializeField] private float angleMultiplier = 1f;
    [SerializeField] private float angleOffset = 0f;

    [Header("Data Validity")]
    [SerializeField] private float staleDataTimeout = 0.5f;

    private Quaternion visualRotation = Quaternion.identity;
    private bool initialized = false;

    void Start()
    {
        receiver = FindObjectOfType<BLEIMUReceiver>();
        rb = GetComponent<Rigidbody>();
        rb.isKinematic = true;
    }

    void FixedUpdate()
    {
        if (receiver == null) return;
        if (receiver.packetCount <= 0) return;
        if (receiver.secondsSinceLastPacket > staleDataTimeout) return;

        float targetRoll = receiver.targetRollAngle * angleMultiplier + angleOffset;
        Quaternion targetRotation = Quaternion.Euler(0f, 0f, targetRoll);

        if (!initialized)
        {
            visualRotation = targetRotation;
            initialized = true;
            rb.MoveRotation(ApplyAxisLocks(visualRotation));
            return;
        }

        float dt = Time.fixedDeltaTime;
        float trackAlpha = 1f - Mathf.Exp(-trackingSpeed * dt);
        visualRotation = Quaternion.Slerp(visualRotation, targetRotation, trackAlpha);

        rb.MoveRotation(ApplyAxisLocks(visualRotation));
    }

    Quaternion ApplyAxisLocks(Quaternion target)
    {
        if (enableRotationX && enableRotationY && enableRotationZ)
            return target;

        Vector3 targetEuler = target.eulerAngles;
        Vector3 currentEuler = rb.rotation.eulerAngles;

        float x = enableRotationX ? targetEuler.x : currentEuler.x;
        float y = enableRotationY ? targetEuler.y : currentEuler.y;
        float z = enableRotationZ ? targetEuler.z : currentEuler.z;

        return Quaternion.Euler(x, y, z);
    }

    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying || !initialized || receiver == null) return;

        float targetRoll = receiver.targetRollAngle * angleMultiplier + angleOffset;
        Quaternion targetRotation = Quaternion.Euler(0f, 0f, targetRoll);

        Gizmos.color = Color.yellow;
        Gizmos.DrawRay(transform.position, targetRotation * Vector3.right * 2f);

        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, visualRotation * Vector3.right * 1.5f);
    }
}