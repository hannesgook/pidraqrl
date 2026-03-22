// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL
// https://github.com/hannesgook/pidraqrl

using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    private BLEIMUReceiver receiver;
    private Rigidbody rb;

    [Header("Complementary Filter")]
    [SerializeField, Range(0.90f, 0.999f)] private float gyroWeight = 0.99f;

    [Header("Drift Correction")]
    [Tooltip("If fused rotation drifts more than this many degrees from absolute, force-correct it.")]
    [SerializeField] private float hardSnapThresholdDeg = 25f;
    [Tooltip("If drift exceeds this softer threshold, blend back more aggressively.")]
    [SerializeField] private float softCorrectionThresholdDeg = 8f;
    [SerializeField, Range(0f, 1f)] private float softCorrectionStrength = 0.15f;
    [SerializeField] private float periodicCheckInterval = 0.1f;

    [Header("Visual Response")]
    [SerializeField, Range(1f, 50f)] private float visualTrackingSpeed = 18f;

    [Header("Axis Locks")]
    [SerializeField] private bool enableRotationX = true;
    [SerializeField] private bool enableRotationY = true;
    [SerializeField] private bool enableRotationZ = true;

    private Quaternion fusedRotation = Quaternion.identity;
    private Quaternion visualRotation = Quaternion.identity;
    private Vector3 smoothedGyroRates = Vector3.zero;
    private bool initialized = false;
    private float timeSinceLastCheck = 0f;
    private Quaternion lastAbsolute = Quaternion.identity;

    void Start()
    {
        receiver = FindObjectOfType<BLEIMUReceiver>();
        rb = GetComponent<Rigidbody>();
        rb.isKinematic = true;
    }

    void FixedUpdate()
    {
        if (receiver == null) return;

        if (!initialized)
        {
            bool hasData = receiver.roll != 0f || receiver.pitch != 0f || receiver.yaw != 0f;
            if (!hasData) return;

            fusedRotation = AbsoluteRotation();
            visualRotation = fusedRotation;
            lastAbsolute = fusedRotation;
            initialized = true;
            return;
        }

        float dt = Time.fixedDeltaTime;
        timeSinceLastCheck += dt;

        Quaternion absoluteTarget = AbsoluteRotation();

        // Low-pass filter on gyro to reduce noise before integration
        const float gyroLPF = 0.4f;
        Vector3 rawGyro = SensorToUnityRates();
        smoothedGyroRates += gyroLPF * (rawGyro - smoothedGyroRates);

        Quaternion gyroDelta = Quaternion.Euler(smoothedGyroRates * dt);
        Quaternion gyroPredicted = fusedRotation * gyroDelta;

        fusedRotation = Quaternion.Slerp(absoluteTarget, gyroPredicted, gyroWeight);

        float driftDeg = Quaternion.Angle(fusedRotation, absoluteTarget);

        if (driftDeg > hardSnapThresholdDeg)
        {
            // Gyro integration error accumulated too far — force snap back
            fusedRotation = absoluteTarget;
            visualRotation = Quaternion.Slerp(visualRotation, absoluteTarget, 0.5f);
            Debug.LogWarning($"[Drone] Hard snap correction — drift was {driftDeg:F1}°");
        }
        else if (driftDeg > softCorrectionThresholdDeg || timeSinceLastCheck >= periodicCheckInterval)
        {
            fusedRotation = Quaternion.Slerp(fusedRotation, absoluteTarget, softCorrectionStrength);

            if (timeSinceLastCheck >= periodicCheckInterval)
                timeSinceLastCheck = 0f;
        }

        lastAbsolute = absoluteTarget;

        float trackAlpha = 1f - Mathf.Exp(-visualTrackingSpeed * dt);
        visualRotation = Quaternion.Slerp(visualRotation, fusedRotation, trackAlpha);

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

    Quaternion AbsoluteRotation()
    {
        return Quaternion.Euler(receiver.pitch, receiver.yaw, receiver.roll);
    }

    Vector3 SensorToUnityRates()
    {
        return new Vector3(receiver.gy, receiver.gz, receiver.gx);
    }

    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying || !initialized) return;
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(transform.position, fusedRotation * Vector3.up * 2f);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(transform.position, AbsoluteRotation() * Vector3.up * 2f);
        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, visualRotation * Vector3.forward * 1.5f);
    }
}