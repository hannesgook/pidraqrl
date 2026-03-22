using UnityEngine;

public class CameraHover : MonoBehaviour
{
    [Header("Orbit Settings")]
    public Transform droneTransform;
    public float orbitRadius = 0.47f;
    public float orbitHeight = 0.9f;
    [Range(0.0f, 10f)]
    public float yawFollowSpeed = 1.5f;

    [Header("Pitch Follow")]
    [Range(0f, 0.5f)]
    public float pitchInfluence = 0.08f;
    [Range(0.5f, 10f)]
    public float pitchFollowSpeed = 2f;
    public float maxPitchHeightOffset = 0.12f;

    [Header("Roll Follow")]
    [Range(0f, 2.0f)]
    public float rollInfluence = 0.12f;
    [Range(0.5f, 10f)]
    public float rollFollowSpeed = 2.5f;
    public float maxRollAngle = 8f;

    [Header("Hover Oscillation")]
    public float maxOffset = 0.2f;
    public float speedX = 0.4f;
    public float speedY = 0.3f;
    public float speedZ = 0.25f;

    private float phaseX;
    private float phaseY;
    private float phaseZ;
    private float currentYaw;
    private float currentPitchOffset;
    private float currentRoll;

    void Start()
    {
        phaseX = Random.Range(0f, Mathf.PI * 2f);
        phaseY = Random.Range(0f, Mathf.PI * 2f);
        phaseZ = Random.Range(0f, Mathf.PI * 2f);

        if (droneTransform != null)
            currentYaw = droneTransform.eulerAngles.y;
    }

    void Update()
    {
        if (droneTransform == null) return;

        // ?? Lazy yaw follow ???????????????????????????????????????????????
        float targetYaw = droneTransform.eulerAngles.y;
        currentYaw = Mathf.LerpAngle(currentYaw, targetYaw, yawFollowSpeed * Time.deltaTime);

        // ?? Orbit position ????????????????????????????????????????????????
        float rad = currentYaw * Mathf.Deg2Rad;
        Vector3 orbitOffset = new Vector3(
            Mathf.Sin(rad) * orbitRadius,
            orbitHeight,
            Mathf.Cos(rad) * orbitRadius
        );

        // ?? Subtle pitch height nudge ?????????????????????????????????????
        float rawPitch = droneTransform.eulerAngles.x;
        float signedPitch = rawPitch > 180f ? rawPitch - 360f : rawPitch;

        float targetPitchOffset = Mathf.Clamp(
            -signedPitch * pitchInfluence,
            -maxPitchHeightOffset,
            maxPitchHeightOffset
        );
        currentPitchOffset = Mathf.Lerp(currentPitchOffset, targetPitchOffset, pitchFollowSpeed * Time.deltaTime);
        orbitOffset.y += currentPitchOffset;

        // ?? Hover oscillation ?????????????????????????????????????????????
        float t = Time.time;
        Vector3 hover = new Vector3(
            Mathf.Sin(t * speedX + phaseX) * maxOffset,
            Mathf.Sin(t * speedY + phaseY) * maxOffset,
            Mathf.Sin(t * speedZ + phaseZ) * (maxOffset * 0.5f)
        );

        transform.position = droneTransform.position + orbitOffset + hover;

        // ?? Look at drone, then apply roll on top ?????????????????????????
        transform.LookAt(droneTransform.position);

        float rawRoll = droneTransform.eulerAngles.z;
        float signedRoll = rawRoll > 180f ? rawRoll - 360f : rawRoll;

        float targetRoll = Mathf.Clamp(
            signedRoll * rollInfluence,
            -maxRollAngle,
            maxRollAngle
        );
        currentRoll = Mathf.Lerp(currentRoll, targetRoll, rollFollowSpeed * Time.deltaTime);

        transform.rotation *= Quaternion.Euler(0f, 0f, -currentRoll);
    }
}