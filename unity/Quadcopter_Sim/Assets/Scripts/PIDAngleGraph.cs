// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL
// https://github.com/hannesgook/pidraqrl

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PIDAngleGraph : MaskableGraphic
{
    public enum AngleSource
    {
        Roll,
        Pitch,
        Yaw
    }

    [Header("Source")]
    public BLEIMUReceiver receiver;
    public AngleSource angleSource = AngleSource.Roll;

    [Header("Sampling")]
    public int maxSamples = 600;
    public float sampleRate = 100f;
    public bool useUnscaledTime = true;

    [Header("Display")]
    public float verticalRange = 45f;
    public float lineThickness = 2f;
    public int maxDisplayPoints = 120;
    public int smoothingWindow = 5;

    [Header("Colors")]
    public Color currentAngleColor = Color.cyan;
    public Color targetAngleColor = Color.red;
    public Color centerLineColor = new Color(1f, 1f, 1f, 0.2f);
    public Color gridColor = new Color(1f, 1f, 1f, 0.08f);

    readonly List<float> currentSamples = new List<float>();
    readonly List<float> targetSamples = new List<float>();

    readonly List<float> smoothedCurrent = new List<float>();
    readonly List<float> smoothedTarget = new List<float>();

    float sampleTimer;

    protected override void Awake()
    {
        base.Awake();
        raycastTarget = false;
    }

    void Update()
    {
        if (receiver == null || maxSamples < 2 || sampleRate <= 0f)
            return;

        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        sampleTimer += dt;

        float interval = 1f / sampleRate;

        while (sampleTimer >= interval)
        {
            sampleTimer -= interval;
            AddSample(GetCurrentAngle(), receiver.targetRollAngle);
            SetVerticesDirty();
        }
    }

    float GetCurrentAngle()
    {
        switch (angleSource)
        {
            case AngleSource.Pitch:
                return receiver.pitch;
            case AngleSource.Yaw:
                return receiver.yaw;
            default:
                return receiver.roll;
        }
    }

    void AddSample(float currentAngle, float targetAngle)
    {
        currentSamples.Add(currentAngle);
        targetSamples.Add(targetAngle);

        while (currentSamples.Count > maxSamples)
            currentSamples.RemoveAt(0);

        while (targetSamples.Count > maxSamples)
            targetSamples.RemoveAt(0);
    }

    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();

        Rect rect = GetPixelAdjustedRect();
        if (rect.width <= 0f || rect.height <= 0f)
            return;

        DrawHorizontalLine(vh, rect, 0f, centerLineColor, 1f);
        DrawGrid(vh, rect);

        int displayPointCount = Mathf.Max(2, Mathf.Min(maxDisplayPoints, Mathf.RoundToInt(rect.width / 4f)));

        BuildDisplaySamples(currentSamples, smoothedCurrent, displayPointCount, Mathf.Max(1, smoothingWindow));
        BuildDisplaySamples(targetSamples, smoothedTarget, displayPointCount, Mathf.Max(1, smoothingWindow));

        if (smoothedCurrent.Count >= 2)
            DrawLineGraph(vh, rect, smoothedCurrent, currentAngleColor, lineThickness);

        if (smoothedTarget.Count >= 2)
            DrawLineGraph(vh, rect, smoothedTarget, targetAngleColor, lineThickness);
    }

    void BuildDisplaySamples(List<float> source, List<float> output, int targetCount, int window)
    {
        output.Clear();

        int count = source.Count;
        if (count == 0 || targetCount <= 0)
            return;

        if (count <= targetCount)
        {
            for (int i = 0; i < count; i++)
            {
                float smoothed = GetWindowAverage(source, i, window);
                output.Add(smoothed);
            }
            return;
        }

        float bucketSize = count / (float)targetCount;

        for (int i = 0; i < targetCount; i++)
        {
            float startF = i * bucketSize;
            float endF = (i + 1) * bucketSize;

            int start = Mathf.FloorToInt(startF);
            int end = Mathf.CeilToInt(endF);

            start = Mathf.Clamp(start, 0, count - 1);
            end = Mathf.Clamp(end, start + 1, count);

            float sum = 0f;
            int samplesInBucket = 0;

            for (int j = start; j < end; j++)
            {
                sum += GetWindowAverage(source, j, window);
                samplesInBucket++;
            }

            output.Add(samplesInBucket > 0 ? sum / samplesInBucket : source[start]);
        }
    }

    float GetWindowAverage(List<float> samples, int centerIndex, int window)
    {
        int half = window / 2;
        int start = Mathf.Max(0, centerIndex - half);
        int end = Mathf.Min(samples.Count - 1, centerIndex + half);

        float sum = 0f;
        int count = 0;

        for (int i = start; i <= end; i++)
        {
            sum += samples[i];
            count++;
        }

        return count > 0 ? sum / count : samples[centerIndex];
    }

    void DrawGrid(VertexHelper vh, Rect rect)
    {
        float[] levels = { -0.75f, -0.5f, -0.25f, 0.25f, 0.5f, 0.75f };

        for (int i = 0; i < levels.Length; i++)
        {
            float value = levels[i] * verticalRange;
            DrawHorizontalLine(vh, rect, value, gridColor, 1f);
        }
    }

    void DrawHorizontalLine(VertexHelper vh, Rect rect, float value, Color color, float thickness)
    {
        float y = ValueToY(rect, value);
        Vector2 a = new Vector2(rect.xMin, y);
        Vector2 b = new Vector2(rect.xMax, y);
        AddThickLine(vh, a, b, thickness, color);
    }

    void DrawLineGraph(VertexHelper vh, Rect rect, List<float> samples, Color color, float thickness)
    {
        int count = samples.Count;
        if (count < 2)
            return;

        float stepX = rect.width / (count - 1);

        Vector2 prev = new Vector2(rect.xMin, ValueToY(rect, samples[0]));

        for (int i = 1; i < count; i++)
        {
            float x = rect.xMin + i * stepX;
            float y = ValueToY(rect, samples[i]);
            Vector2 current = new Vector2(x, y);
            AddThickLine(vh, prev, current, thickness, color);
            prev = current;
        }
    }

    float ValueToY(Rect rect, float value)
    {
        float normalized = Mathf.InverseLerp(-verticalRange, verticalRange, Mathf.Clamp(value, -verticalRange, verticalRange));
        return Mathf.Lerp(rect.yMin, rect.yMax, normalized);
    }

    void AddThickLine(VertexHelper vh, Vector2 a, Vector2 b, float thickness, Color color)
    {
        Vector2 dir = b - a;
        float len = dir.magnitude;
        if (len <= 0.0001f)
            return;

        dir /= len;
        Vector2 normal = new Vector2(-dir.y, dir.x) * (thickness * 0.5f);

        int startIndex = vh.currentVertCount;

        UIVertex vert = UIVertex.simpleVert;
        vert.color = color;

        vert.position = a - normal;
        vh.AddVert(vert);

        vert.position = a + normal;
        vh.AddVert(vert);

        vert.position = b + normal;
        vh.AddVert(vert);

        vert.position = b - normal;
        vh.AddVert(vert);

        vh.AddTriangle(startIndex + 0, startIndex + 1, startIndex + 2);
        vh.AddTriangle(startIndex + 2, startIndex + 3, startIndex + 0);
    }

    public void ClearGraph()
    {
        currentSamples.Clear();
        targetSamples.Clear();
        smoothedCurrent.Clear();
        smoothedTarget.Clear();
        SetVerticesDirty();
    }
}