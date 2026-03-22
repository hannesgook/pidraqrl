// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL
// https://github.com/hannesgook/pidraqrl

using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class BLEDroneController : MonoBehaviour
{
    private BLEIMUReceiver receiver;
    private Rigidbody rb;

    [Header("Visual Smoothing")]
    [Tooltip("How quickly the visual rotation tracks the sensor. Lower = smoother, higher = more responsive.")]
    [SerializeField, Range(1f, 50f)] private float trackingSpeed = 12f;

    [Header("Axis Locks")]
    [SerializeField] private bool enableRotationX = true;
    [SerializeField] private bool enableRotationY = true;
    [SerializeField] private bool enableRotationZ = true;

    [Header("Motor Vibration")]
    [SerializeField] private bool enableVibration = true;
    [SerializeField] private float vibrationAmplitude = 0.005f;
    [SerializeField] private float vibrationFrequency = 60f;

    [Header("Prop Wash")]
    [SerializeField] private bool enablePropWash = true;
    [SerializeField] private float propWashStrength = 0.008f;
    [SerializeField] private float propWashThreshold = 45f;

    [Header("Data Validity")]
    [SerializeField] private float staleDataTimeout = 0.5f;

    [Header("Secondary Object")]
    [Tooltip("A child GameObject that will mirror the drone's world-space rotation, pivoting around the drone's (parent) axes rather than its own local axes.")]
    [SerializeField] private Transform secondaryObject;

    private Quaternion visualRotation = Quaternion.identity;
    private Vector3 smoothedGyroRates = Vector3.zero;
    private bool initialized = false;
    private float estimatedThrottle = 0f;
    private float vibPhase = 0f;

    public float lookahead = 0.2f;

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

        Quaternion sensorRotation = Quaternion.Euler(receiver.pitch, -receiver.yaw, receiver.roll);

        if (!initialized)
        {
            visualRotation = sensorRotation;
            initialized = true;
            return;
        }

        float dt = Time.fixedDeltaTime;

        // Map gyro rates to Unity axes (deg/s)
        Vector3 rawGyro = new Vector3(receiver.gy, -receiver.gz, receiver.gx);

        // Smooth gyro for effects only
        const float gyroLPF = 0.2f;
        smoothedGyroRates += gyroLPF * (rawGyro - smoothedGyroRates);

        // Predict a small step ahead of sensorRotation using gyro rates.
        Quaternion gyroDelta = Quaternion.Euler(rawGyro * lookahead);
        Quaternion predictedRotation = sensorRotation * gyroDelta;

        // Prop wash
        Quaternion washOffset = Quaternion.identity;
        if (enablePropWash)
        {
            float gyroMag = smoothedGyroRates.magnitude;
            if (gyroMag > propWashThreshold)
            {
                float washScale = Mathf.InverseLerp(propWashThreshold, propWashThreshold * 3f, gyroMag);
                Vector3 washNoise = new Vector3(
                    Mathf.PerlinNoise(Time.time * 17.3f, 0f) - 0.5f,
                    Mathf.PerlinNoise(0f, Time.time * 13.7f) - 0.5f,
                    Mathf.PerlinNoise(Time.time * 11.1f, Time.time * 7.3f) - 0.5f
                ) * propWashStrength * washScale;
                washOffset = Quaternion.Euler(washNoise);
            }
        }

        // Motor vibration
        float gyroActivity = Mathf.Clamp01(smoothedGyroRates.magnitude / 180f);
        estimatedThrottle = Mathf.Lerp(estimatedThrottle, gyroActivity, dt * 3f);

        Quaternion vibOffset = Quaternion.identity;
        if (enableVibration && estimatedThrottle > 0.05f)
        {
            vibPhase += vibrationFrequency * dt * Mathf.PI * 2f;
            float vib = Mathf.Sin(vibPhase) * vibrationAmplitude * estimatedThrottle;
            vibOffset = Quaternion.Euler(vib, vib * 0.3f, vib);
        }

        // Slerp toward the gyro-predicted target (still angle-anchored, no drift)
        Quaternion targetWithEffects = predictedRotation * washOffset * vibOffset;
        float trackAlpha = 1f - Mathf.Exp(-trackingSpeed * dt);
        visualRotation = Quaternion.Slerp(visualRotation, targetWithEffects, trackAlpha);

        Quaternion finalRotation = ApplyAxisLocks(visualRotation);
        rb.MoveRotation(finalRotation);

        if (secondaryObject != null)
        {
            Quaternion worldRotation = finalRotation;
            Quaternion parentInverse = transform.parent != null
                ? Quaternion.Inverse(transform.parent.rotation)
                : Quaternion.identity;

            secondaryObject.localRotation = Quaternion.identity;
        }
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

        Quaternion sensorRot = Quaternion.Euler(receiver.pitch, -receiver.yaw, receiver.roll);

        Gizmos.color = Color.green;
        Gizmos.DrawRay(transform.position, sensorRot * Vector3.up * 2f);

        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, visualRotation * Vector3.forward * 1.5f);
    }
}