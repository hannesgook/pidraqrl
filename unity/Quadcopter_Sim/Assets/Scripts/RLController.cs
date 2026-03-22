// Copyright (c) 2025 Hannes Göök
// MIT License - PidraQRL
// https://github.com/hannesgook/pidraqrl

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class RLController : MonoBehaviour
{
    [Header("UI (optional — fallback OnGUI buttons shown if unassigned)")]
    public Button startButton;
    public Button stopButton;
    public Text statusText;

    [Header("Network")]
    public string pythonIP = "127.0.0.1";
    public int cmdPort = 5006;
    public int statusPort = 5007;

    string _statusLabel = "STATUS:IDLE";
    readonly object _statusLock = new object();

    UdpClient _statusClient;
    Thread _statusThread;
    bool _quitting;


    void Start()
    {
        if (startButton != null) startButton.onClick.AddListener(StartTraining);
        if (stopButton != null) stopButton.onClick.AddListener(StopTraining);

        _statusClient = new UdpClient(statusPort);
        _statusClient.Client.ReceiveTimeout = 500;
        _statusThread = new Thread(StatusReceiveLoop) { IsBackground = true };
        _statusThread.Start();

        Debug.Log($"[RLController] Ready.  cmd→{pythonIP}:{cmdPort}  status←:{statusPort}");
    }

    void Update()
    {
        if (statusText != null)
        {
            lock (_statusLock)
                statusText.text = _statusLabel;
        }
    }

    void OnDestroy()
    {
        _quitting = true;
        _statusClient?.Close();
        _statusThread?.Join(300);
    }

    void OnApplicationQuit() => OnDestroy();

    void OnGUI()
    {
        if (startButton != null && stopButton != null) return;
        GUIStyle btnStyle = new GUIStyle(GUI.skin.button) { fontSize = 18 };
        float x = 20, y = 70, w = 200, h = 44, pad = 8;

        if (GUI.Button(new Rect(x, y, w - pad, h), "Start Training", btnStyle))
            StartTraining();
        if (GUI.Button(new Rect(x + w, y, w - pad, h), "Stop Training", btnStyle))
            StopTraining();
    }
    public void StartTraining()
    {
        SendCommand("START_TRAINING");
        Debug.Log("[RLController] Sent START_TRAINING");
    }

    public void StopTraining()
    {
        SendCommand("STOP_TRAINING");
        Debug.Log("[RLController] Sent STOP_TRAINING");
    }

    void SendCommand(string cmd)
    {
        try
        {
            using var sock = new UdpClient();
            byte[] data = Encoding.ASCII.GetBytes(cmd);
            sock.Send(data, data.Length, pythonIP, cmdPort);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[RLController] Send failed: {e.Message}");
        }
    }

    void StatusReceiveLoop()
    {
        var remote = new IPEndPoint(IPAddress.Any, 0);
        while (!_quitting)
        {
            try
            {
                byte[] data = _statusClient.Receive(ref remote);
                string msg = Encoding.ASCII.GetString(data).Trim();
                lock (_statusLock)
                    _statusLabel = msg;
                // Also log to Console so you see it without a UI
                UnityEngine.Debug.Log($"[RLController] ← Python: {msg}");
            }
            catch (SocketException) { /* timeout — normal */ }
            catch (ObjectDisposedException) { break; }
        }
    }
}