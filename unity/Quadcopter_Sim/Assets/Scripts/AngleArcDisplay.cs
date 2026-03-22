// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL
// https://github.com/hannesgook/pidraqrl

using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class AngleArcDisplay : MonoBehaviour
{
    [Header("Source")]
    [Tooltip("The transform whose rotation angle to display. If null, uses BLEDroneController's object.")]
    [SerializeField] private Transform target;

    [Header("Anchor")]
    [Tooltip("The transform used as the center/origin for the arc, reference line, and label. If null, uses this transform.")]
    [SerializeField] private Transform arcAnchor;

    [Header("Angle Settings")]
    [SerializeField] private AngleAxis angleAxis = AngleAxis.Pitch;
    [SerializeField] private float referenceAngle = 0f;

    [Header("Arc Appearance")]
    [SerializeField] private float arcRadius = 1.5f;
    [SerializeField] private int arcSegments = 48;
    [SerializeField] private float arcWidth = 0.03f;
    [SerializeField] private Color arcColor = new Color(0f, 0.85f, 1f, 0.9f);
    [SerializeField] private Color warningArcColor = new Color(1f, 0.4f, 0.2f, 0.9f);
    [Tooltip("Arc turns warning color when angle exceeds this threshold or goes below its negative.")]
    [SerializeField] private float angleThreshold = 45f;

    [Header("Reference Line")]
    [SerializeField] private bool showReferenceLine = true;
    [SerializeField] private Color referenceLineColor = new Color(1f, 1f, 1f, 0.3f);

    // ── NEW ──────────────────────────────────────────────────────────────────
    [Header("Target Roll Line")]
    [SerializeField] private bool showTargetLine = true;
    [SerializeField] private Color targetLineColor = new Color(1f, 0.85f, 0f, 0.9f);   // gold
    // ─────────────────────────────────────────────────────────────────────────

    [Header("Text Label")]
    [SerializeField] private bool showLabel = true;
    [SerializeField] private float labelFontSize = 0.12f;
    [SerializeField] private Color labelColor = Color.white;
    [SerializeField] private float labelOffsetFromArc = 0.15f;
    [SerializeField] private string formatString = "0.0";
    [SerializeField] private string suffix = "°";

    [Header("Smoothing")]
    [SerializeField, Range(1f, 30f)] private float smoothingSpeed = 12f;

    public enum AngleAxis { Pitch, Yaw, Roll }

    private LineRenderer arcLine;
    private LineRenderer refLine;

    // ── NEW ──────────────────────────────────────────────────────────────────
    private LineRenderer targetLine;
    private BLEIMUReceiver bleReceiver;
    // ─────────────────────────────────────────────────────────────────────────

    private TextMesh label;

    private Transform Anchor => arcAnchor != null ? arcAnchor : transform;

    private bool IsOverThreshold(float angleDeg) => Mathf.Abs(angleDeg) > angleThreshold;

    private float displayedAngle = 0f;
    private float displayedTargetAngle = 0f;  // NEW

    void Start()
    {
        arcLine = GetComponent<LineRenderer>();
        ConfigureLineRenderer(arcLine, arcColor, arcWidth);

        if (showReferenceLine)
        {
            GameObject refObj = new GameObject("ReferenceLine");
            refObj.transform.SetParent(transform, false);
            refLine = refObj.AddComponent<LineRenderer>();
            ConfigureLineRenderer(refLine, referenceLineColor, arcWidth * 0.5f);
            refLine.positionCount = 2;
        }

        // ── NEW ──────────────────────────────────────────────────────────────
        if (showTargetLine)
        {
            GameObject tlObj = new GameObject("TargetRollLine");
            tlObj.transform.SetParent(transform, false);
            targetLine = tlObj.AddComponent<LineRenderer>();
            ConfigureLineRenderer(targetLine, targetLineColor, arcWidth * 0.75f);
            targetLine.positionCount = 2;
        }

        bleReceiver = FindObjectOfType<BLEIMUReceiver>();
        // ─────────────────────────────────────────────────────────────────────

        if (showLabel)
        {
            GameObject labelObj = new GameObject("AngleLabel");
            labelObj.transform.SetParent(transform, false);
            label = labelObj.AddComponent<TextMesh>();
            label.characterSize = labelFontSize;
            label.anchor = TextAnchor.MiddleCenter;
            label.alignment = TextAlignment.Center;
            label.color = labelColor;
            label.fontSize = 48;
            label.text = "";
        }

        if (target == null)
        {
            var drone = FindObjectOfType<BLEDroneController>();
            if (drone != null) target = drone.transform;
        }
    }

    void LateUpdate()
    {
        if (target == null) return;

        float rawAngle = GetCurrentAngle();
        displayedAngle = Mathf.Lerp(displayedAngle, rawAngle, 1f - Mathf.Exp(-smoothingSpeed * Time.deltaTime));
        displayedTargetAngle = Mathf.Lerp(displayedTargetAngle, bleReceiver != null ? bleReceiver.targetRollAngle : displayedTargetAngle, 1f - Mathf.Exp(-smoothingSpeed * Time.deltaTime));  // NEW

        DrawArc(displayedAngle);
        UpdateReferenceLine();
        UpdateLabel(displayedAngle);

        // ── NEW ──────────────────────────────────────────────────────────────
        UpdateTargetLine();
        // ─────────────────────────────────────────────────────────────────────
    }

    // ── NEW ──────────────────────────────────────────────────────────────────
    void UpdateTargetLine()
    {
        if (targetLine == null || bleReceiver == null) return;

        Transform anchor = Anchor;
        float rad = (referenceAngle + displayedTargetAngle) * Mathf.Deg2Rad;  // was bleReceiver.targetRollAngle
        Vector3 dir = new Vector3(Mathf.Sin(rad), Mathf.Cos(rad), 0f);

        targetLine.SetPosition(0, anchor.position);
        targetLine.SetPosition(1, anchor.TransformPoint(dir * arcRadius * 1.15f));
    }
    // ─────────────────────────────────────────────────────────────────────────

    float GetCurrentAngle()
    {
        Vector3 euler = target.rotation.eulerAngles;
        float raw = angleAxis switch
        {
            AngleAxis.Pitch => euler.x,
            AngleAxis.Yaw => euler.y,
            AngleAxis.Roll => euler.z,
            _ => 0f
        };

        float signed = NormalizeAngle(raw);
        return NormalizeAngle(signed - referenceAngle);
    }

    static float NormalizeAngle(float a)
    {
        while (a > 180f) a -= 360f;
        while (a < -180f) a += 360f;
        return a;
    }

    void DrawArc(float angleDeg)
    {
        float absAngle = Mathf.Abs(angleDeg);
        int count = Mathf.Max(2, Mathf.CeilToInt(arcSegments * (absAngle / 360f)));
        arcLine.positionCount = count;

        float startRad = referenceAngle * Mathf.Deg2Rad;
        float endRad = (referenceAngle + angleDeg) * Mathf.Deg2Rad;

        Transform anchor = Anchor;

        for (int i = 0; i < count; i++)
        {
            float t = count > 1 ? (float)i / (count - 1) : 0f;
            float rad = Mathf.Lerp(startRad, endRad, t);
            Vector3 localPos = new Vector3(Mathf.Sin(rad), Mathf.Cos(rad), 0f) * arcRadius;
            arcLine.SetPosition(i, anchor.TransformPoint(localPos));
        }

        Color c = IsOverThreshold(angleDeg) ? warningArcColor : arcColor;
        arcLine.startColor = c;
        arcLine.endColor = c;
    }

    void UpdateReferenceLine()
    {
        if (refLine == null) return;

        Transform anchor = Anchor;
        float rad = referenceAngle * Mathf.Deg2Rad;
        Vector3 dir = new Vector3(Mathf.Sin(rad), Mathf.Cos(rad), 0f);

        refLine.SetPosition(0, anchor.position);
        refLine.SetPosition(1, anchor.TransformPoint(dir * arcRadius * 1.1f));
    }

    void UpdateLabel(float angleDeg)
    {
        if (label == null) return;

        label.text = angleDeg.ToString(formatString) + suffix;

        Transform anchor = Anchor;
        float midRad = (referenceAngle + angleDeg * 0.5f) * Mathf.Deg2Rad;
        float labelRadius = arcRadius + labelOffsetFromArc;
        Vector3 localPos = new Vector3(Mathf.Sin(midRad), Mathf.Cos(midRad), 0f) * labelRadius;
        label.transform.position = anchor.TransformPoint(localPos);

        if (Camera.main != null)
        {
            label.transform.rotation = Quaternion.LookRotation(
                label.transform.position - Camera.main.transform.position,
                Camera.main.transform.up
            );
        }

        label.color = IsOverThreshold(angleDeg) ? warningArcColor : labelColor;
    }

    void ConfigureLineRenderer(LineRenderer lr, Color color, float width)
    {
        lr.useWorldSpace = true;
        lr.startWidth = width;
        lr.endWidth = width;
        lr.startColor = color;
        lr.endColor = color;
        lr.numCornerVertices = 4;
        lr.numCapVertices = 4;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;

        Transform anchor = arcAnchor != null ? arcAnchor : transform;
        Vector3 pos = anchor.position;

        int steps = 36;
        for (int i = 0; i < steps; i++)
        {
            float a1 = (i / (float)steps) * Mathf.PI * 2f;
            float a2 = ((i + 1) / (float)steps) * Mathf.PI * 2f;

            Vector3 p1 = pos + anchor.TransformDirection(new Vector3(Mathf.Sin(a1), Mathf.Cos(a1), 0f)) * arcRadius;
            Vector3 p2 = pos + anchor.TransformDirection(new Vector3(Mathf.Sin(a2), Mathf.Cos(a2), 0f)) * arcRadius;

            Gizmos.DrawLine(p1, p2);
        }
    }
}