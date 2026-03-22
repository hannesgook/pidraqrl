// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL
// https://github.com/hannesgook/pidraqrl

using UnityEngine;
using System.Collections.Generic;

public class CinematicCameraPath : MonoBehaviour
{
    public enum CameraMode { Path, Orbit, Fixed, Walk }

    [System.Serializable]
    public class PathNode
    {
        public Transform point;
        public Transform lookAtTarget;
        [Tooltip("Seconds to travel FROM the previous node TO this one. Ignored on the first node.")]
        [Range(0.1f, 60f)]
        public float duration = 3f;
        [Tooltip("How much to ease in/out when arriving at this node.")]
        [Range(0f, 0.5f)]
        public float easeStrength = 0.4f;
    }

    [Header("Path")]
    public List<PathNode> nodes = new List<PathNode>();

    [Header("Interpolation")]
    public bool useSplineInterpolation = true;
    [Tooltip("How smoothly the camera transitions between different look-at targets.")]
    [Range(0.01f, 1f)]
    public float lookAtTransitionSmoothing = 0.05f;

    [Header("Organic Motion")]
    public bool enableOrganicMotion = true;
    [Range(0f, 0.15f)]
    public float positionNoiseAmplitude = 0.04f;
    [Range(0f, 2f)]
    public float positionNoiseSpeed = 0.6f;
    [Range(0f, 1f)]
    public float rotationNoiseAmplitude = 0.3f;
    [Range(0f, 2f)]
    public float rotationNoiseSpeed = 0.4f;

    [Header("Playback")]
    public bool playOnStart = true;
    public bool loop = false;

    [Header("Orbit Mode")]
    public Transform orbitTarget;
    [Range(0.5f, 50f)]
    public float orbitRadius = 5f;
    [Tooltip("Angle above the horizontal plane in degrees.")]
    [Range(-80f, 80f)]
    public float orbitElevationAngle = 25f;
    [Range(1f, 60f)]
    public float orbitSpeed = 10f;

    [Header("Fixed Mode")]
    public Transform fixedPosition;
    public Transform fixedLookAtTarget;

    [Header("Walk Mode")]
    [Range(0.5f, 10f)]
    public float walkSpeed = 1.5f;
    [Range(0.1f, 5f)]
    public float mouseSensitivity = 1.5f;
    [Tooltip("Fixed height above the ground (uses raycast down).")]
    [Range(0.1f, 10f)]
    public float walkHeight = 1.6f;
    [Tooltip("Layer mask for ground detection.")]
    public LayerMask groundLayer = ~0;
    [Tooltip("Vertical fly speed in meters per second.")]
    public float flySpeed = 0.1f;
    [Tooltip("Minimum Y position when free-flying.")]
    public float flyMinY = 0.1f;

    [Header("Mode")]
    [SerializeField] private CameraMode currentMode = CameraMode.Path;

    // Path state
    private int currentSegment = 0;
    private float segmentT = 0f;
    private bool isPlaying = false;
    private Vector3 currentLookDirection;

    // Orbit state
    private float orbitAngle = 0f;

    // Walk state
    private float walkYaw = 0f;
    private float walkPitch = 0f;
    private bool cursorLocked = false;
    private bool walkCrouching = false;
    private bool walkFlying = false;

    private float seedX, seedY, seedZ, seedRX, seedRY;

    public CameraMode CurrentMode => currentMode;

    void Start()
    {
        seedX = Random.Range(0f, 100f);
        seedY = Random.Range(0f, 100f);
        seedZ = Random.Range(0f, 100f);
        seedRX = Random.Range(0f, 100f);
        seedRY = Random.Range(0f, 100f);

        if (nodes.Count >= 2 && nodes[0].point != null)
        {
            transform.position = nodes[0].point.position;

            Transform firstTarget = GetLookTarget(0);
            if (firstTarget != null)
                currentLookDirection = (firstTarget.position - transform.position).normalized;
        }

        if (playOnStart)
            Play();
    }

    public void Play()
    {
        if (nodes.Count < 2)
        {
            Debug.LogWarning("CinematicCameraPath: Need at least 2 nodes.");
            return;
        }
        currentSegment = 0;
        segmentT = 0f;
        isPlaying = true;
    }

    public void Stop() => isPlaying = false;

    public void SetMode(CameraMode mode)
    {
        // Unlock cursor when leaving Walk mode
        if (currentMode == CameraMode.Walk)
            SetCursorLocked(false);

        currentMode = mode;

        if (mode == CameraMode.Path)
        {
            Play();
        }
        else if (mode == CameraMode.Fixed && fixedPosition != null)
        {
            transform.position = fixedPosition.position;
            transform.rotation = fixedPosition.rotation;
            if (fixedLookAtTarget != null)
                currentLookDirection = (fixedLookAtTarget.position - transform.position).normalized;
        }
        else if (mode == CameraMode.Walk)
        {
            // Initialize yaw/pitch from current camera rotation
            Vector3 euler = transform.eulerAngles;
            walkYaw = euler.y;
            walkPitch = euler.x;
            if (walkPitch > 180f) walkPitch -= 360f;
            walkCrouching = false;
            walkFlying = false;
            SetCursorLocked(true);
        }
    }

    public void CycleMode()
    {
        int next = ((int)currentMode + 1) % 4;
        SetMode((CameraMode)next);
    }

    void SetCursorLocked(bool locked)
    {
        cursorLocked = locked;
        Cursor.lockState = locked ? CursorLockMode.Locked : CursorLockMode.None;
        Cursor.visible = !locked;
    }

    /// Returns the look target for a node, walking backwards to find one if null.
    Transform GetLookTarget(int nodeIndex)
    {
        for (int i = nodeIndex; i >= 0; i--)
        {
            if (nodes[i].lookAtTarget != null)
                return nodes[i].lookAtTarget;
        }
        return null;
    }

    void Update()
    {
        // Space always cycles modes
        if (Input.GetKeyDown(KeyCode.Space))
            CycleMode();

        if (currentMode == CameraMode.Walk)
        {
            if (Input.GetKeyDown(KeyCode.Escape))
                SetCursorLocked(!cursorLocked);

            if (!cursorLocked && Input.GetMouseButtonDown(0))
                SetCursorLocked(true);

            if (Input.GetKeyDown(KeyCode.Tab))
            {
                walkFlying = !walkFlying;
                Debug.Log("[CinematicCamera] Free-fly: " + walkFlying);
            }
        }
    }

    void LateUpdate()
    {
        switch (currentMode)
        {
            case CameraMode.Path:
                UpdatePath();
                break;
            case CameraMode.Orbit:
                UpdateOrbit();
                break;
            case CameraMode.Fixed:
                UpdateFixed();
                break;
            case CameraMode.Walk:
                UpdateWalk();
                break;
        }
    }
    void UpdatePath()
    {
        if (!isPlaying || nodes.Count < 2)
            return;

        float duration = nodes[currentSegment + 1].duration;
        segmentT += Time.deltaTime / Mathf.Max(duration, 0.001f);

        while (segmentT >= 1f && isPlaying)
        {
            segmentT -= 1f;
            currentSegment++;

            if (currentSegment >= nodes.Count - 1)
            {
                if (loop)
                {
                    currentSegment = 0;
                }
                else
                {
                    currentSegment = nodes.Count - 2;
                    segmentT = 1f;
                    isPlaying = false;
                }
            }
        }

        float easeStr = nodes[currentSegment + 1].easeStrength;
        float easedT = ApplyEasing(segmentT, easeStr);

        Vector3 basePos;
        if (useSplineInterpolation)
            basePos = CatmullRom(currentSegment, easedT);
        else
            basePos = Vector3.Lerp(nodes[currentSegment].point.position, nodes[currentSegment + 1].point.position, easedT);

        if (enableOrganicMotion)
            basePos += GetPositionNoise();

        transform.position = basePos;

        // Look at
        Transform targetA = GetLookTarget(currentSegment);
        Transform targetB = GetLookTarget(currentSegment + 1);

        Vector3 desiredLookPos;
        if (targetA != null && targetB != null)
        {
            float focusBlend = segmentT * segmentT * (3f - 2f * segmentT);
            desiredLookPos = Vector3.Lerp(targetA.position, targetB.position, focusBlend);
        }
        else if (targetB != null)
            desiredLookPos = targetB.position;
        else if (targetA != null)
            desiredLookPos = targetA.position;
        else
            desiredLookPos = transform.position + currentLookDirection;

        ApplyLookAt(desiredLookPos);
    }

    void UpdateOrbit()
    {
        if (orbitTarget == null)
        {
            Debug.LogWarning("CinematicCameraPath: Orbit mode requires an orbitTarget.");
            return;
        }

        orbitAngle += orbitSpeed * Time.deltaTime;

        float elevRad = orbitElevationAngle * Mathf.Deg2Rad;
        float horizontalDist = orbitRadius * Mathf.Cos(elevRad);
        float verticalDist = orbitRadius * Mathf.Sin(elevRad);

        float rad = orbitAngle * Mathf.Deg2Rad;
        Vector3 offset = new Vector3(
            Mathf.Sin(rad) * horizontalDist,
            verticalDist,
            Mathf.Cos(rad) * horizontalDist
        );

        transform.position = orbitTarget.position + offset;
        transform.LookAt(orbitTarget);
    }
    void UpdateFixed()
    {
        if (fixedPosition == null)
        {
            Debug.LogWarning("CinematicCameraPath: Fixed mode requires a fixedPosition transform.");
            return;
        }

        Vector3 basePos = fixedPosition.position;

        //if (enableOrganicMotion)
        //    basePos += GetPositionNoise();

        transform.position = basePos;

        if (fixedLookAtTarget != null)
        {
            ApplyLookAt(fixedLookAtTarget.position);
        }
        else
        {
            Quaternion baseRot = fixedPosition.rotation;
            //if (enableOrganicMotion)
            //    baseRot *= GetRotationNoise();
            transform.rotation = baseRot;
        }
    }

    void UpdateWalk()
    {
        if (cursorLocked)
        {
            walkYaw += Input.GetAxis("Mouse X") * mouseSensitivity;
            walkPitch -= Input.GetAxis("Mouse Y") * mouseSensitivity;
            walkPitch = Mathf.Clamp(walkPitch, -85f, 85f);
        }

        Quaternion rotation = Quaternion.Euler(walkPitch, walkYaw, 0f);

        // Horizontal movement is always relative to yaw only (no pitch tilt on WASD)
        Vector3 forward = Quaternion.Euler(0f, walkYaw, 0f) * Vector3.forward;
        Vector3 right = Quaternion.Euler(0f, walkYaw, 0f) * Vector3.right;

        float h = Input.GetAxisRaw("Horizontal");
        float v = Input.GetAxisRaw("Vertical");
        Vector3 moveDir = (forward * v + right * h).normalized;

        Vector3 pos;

        if (walkFlying)
        {
            Vector3 camForward = Quaternion.Euler(walkPitch, walkYaw, 0f) * Vector3.forward;
            Vector3 camRight = Quaternion.Euler(0f, walkYaw, 0f) * Vector3.right;

            float eqInput = 0f;
            if (Input.GetKey(KeyCode.E)) eqInput = 1f;
            if (Input.GetKey(KeyCode.Q)) eqInput = -1f;

            Vector3 flyMove = (camForward * v + camRight * h).normalized;
            pos = transform.position + flyMove * flySpeed * Time.deltaTime + transform.up * eqInput * flySpeed * Time.deltaTime;
            pos.y = Mathf.Max(pos.y, flyMinY);
        }
        else
        {
            pos = transform.position + moveDir * walkSpeed * Time.deltaTime;

            if (Input.GetKeyDown(KeyCode.LeftShift) || Input.GetKeyDown(KeyCode.RightShift) ||
                Input.GetKeyDown(KeyCode.LeftControl) || Input.GetKeyDown(KeyCode.RightControl) ||
                Input.GetKeyDown(KeyCode.C))
                walkCrouching = !walkCrouching;

            float currentHeight = walkCrouching ? walkHeight * 0.5f : walkHeight;

            if (Physics.Raycast(pos + Vector3.up * 50f, Vector3.down, out RaycastHit hit, 100f, groundLayer))
                pos.y = hit.point.y + currentHeight;
            else
                pos.y = currentHeight;
        }

        transform.position = pos;
        transform.rotation = rotation;
    }

    void ApplyLookAt(Vector3 desiredLookPos)
    {
        Vector3 desiredDir = (desiredLookPos - transform.position).normalized;
        float smoothFactor = 1f - Mathf.Exp(-10f * (1f - lookAtTransitionSmoothing) * Time.deltaTime);
        currentLookDirection = Vector3.Slerp(currentLookDirection, desiredDir, smoothFactor);
        Quaternion baseRot = Quaternion.LookRotation(currentLookDirection);

        if (enableOrganicMotion)
            baseRot *= GetRotationNoise();

        transform.rotation = baseRot;
    }

    Vector3 GetPositionNoise()
    {
        float t = Time.time * positionNoiseSpeed;
        return new Vector3(
            Mathf.Sin(t * 1.0f + seedX) + Mathf.Sin(t * 2.3f + seedX * 0.7f) * 0.5f,
            Mathf.Sin(t * 0.7f + seedY) + Mathf.Sin(t * 1.8f + seedY * 0.7f) * 0.5f,
            Mathf.Sin(t * 1.3f + seedZ) + Mathf.Sin(t * 0.9f + seedZ * 0.7f) * 0.5f
        ) * positionNoiseAmplitude;
    }

    Quaternion GetRotationNoise()
    {
        float rt = Time.time * rotationNoiseSpeed;
        float noisePitch = (Mathf.Sin(rt * 1.1f + seedRX) + Mathf.Sin(rt * 2.7f + seedRX * 0.5f) * 0.4f) * rotationNoiseAmplitude;
        float noiseYaw = (Mathf.Sin(rt * 0.8f + seedRY) + Mathf.Sin(rt * 2.1f + seedRY * 0.5f) * 0.4f) * rotationNoiseAmplitude;
        return Quaternion.Euler(noisePitch, noiseYaw, 0f);
    }

    float ApplyEasing(float t, float strength)
    {
        if (strength <= 0f) return t;
        float smooth = t * t * (3f - 2f * t);
        return Mathf.Lerp(t, smooth, strength * 2f);
    }

    Vector3 CatmullRom(int segIndex, float t)
    {
        int count = nodes.Count;
        Vector3 p0 = nodes[Mathf.Max(segIndex - 1, 0)].point.position;
        Vector3 p1 = nodes[segIndex].point.position;
        Vector3 p2 = nodes[Mathf.Min(segIndex + 1, count - 1)].point.position;
        Vector3 p3 = nodes[Mathf.Min(segIndex + 2, count - 1)].point.position;

        float tt = t * t;
        float ttt = tt * t;

        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * tt +
            (-p0 + 3f * p1 - 3f * p2 + p3) * ttt
        );
    }

    void OnDrawGizmos()
    {
        if (nodes == null || nodes.Count < 2) return;

        // Draw spline
        Gizmos.color = Color.cyan;
        for (int seg = 0; seg < nodes.Count - 1; seg++)
        {
            if (nodes[seg].point == null || nodes[seg + 1].point == null) continue;

            Vector3 prev = useSplineInterpolation ? CatmullRom(seg, 0f) : nodes[seg].point.position;
            for (int i = 1; i <= 20; i++)
            {
                float t = i / 20f;
                Vector3 curr = useSplineInterpolation ? CatmullRom(seg, t) : Vector3.Lerp(nodes[seg].point.position, nodes[seg + 1].point.position, t);
                Gizmos.DrawLine(prev, curr);
                prev = curr;
            }
        }

        // Draw nodes and their look-at connections
        for (int i = 0; i < nodes.Count; i++)
        {
            if (nodes[i].point == null) continue;

            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(nodes[i].point.position, 0.02f);

            if (nodes[i].lookAtTarget != null)
            {
                Gizmos.color = new Color(1f, 0.3f, 0.3f, 0.4f);
                Gizmos.DrawLine(nodes[i].point.position, nodes[i].lookAtTarget.position);
                Gizmos.DrawWireSphere(nodes[i].lookAtTarget.position, 0.02f);
            }
        }

        // Draw orbit preview
        if (orbitTarget != null)
        {
            Gizmos.color = new Color(0.3f, 1f, 0.3f, 0.3f);
            float elevRad = orbitElevationAngle * Mathf.Deg2Rad;
            float hDist = orbitRadius * Mathf.Cos(elevRad);
            float vDist = orbitRadius * Mathf.Sin(elevRad);
            Vector3 prevOrbit = orbitTarget.position + new Vector3(0f, vDist, hDist);
            for (int i = 1; i <= 36; i++)
            {
                float angle = (i / 36f) * Mathf.PI * 2f;
                Vector3 pt = orbitTarget.position + new Vector3(Mathf.Sin(angle) * hDist, vDist, Mathf.Cos(angle) * hDist);
                Gizmos.DrawLine(prevOrbit, pt);
                prevOrbit = pt;
            }
        }
    }
}