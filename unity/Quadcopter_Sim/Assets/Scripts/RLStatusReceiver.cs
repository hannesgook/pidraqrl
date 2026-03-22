// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL Project
// https://github.com/hannesgook/pidraqrl

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class RLStatusReceiver : MonoBehaviour
{
    [Header("Network")]
    public int statusPort = 5007;

    [Header("UI")]
    public Slider qualitySlider;
    public TextMeshProUGUI qualityLabel;
    public TextMeshProUGUI statusLabel;

    // Thread-safe
    float _pendingQuality = 50f;
    string _pendingStatus = "IDLE";
    bool _dirty = false;
    readonly object _lock = new object();

    UdpClient _udp;
    Thread _thread;
    bool _quit;

    void Start()
    {
        if (!qualitySlider) Debug.LogError("[RLStatus] qualitySlider not assigned!");
        if (!qualityLabel) Debug.LogError("[RLStatus] qualityLabel not assigned!");
        if (!statusLabel) Debug.LogError("[RLStatus] statusLabel not assigned!");

        _udp = new UdpClient();
        _udp.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
        _udp.Client.Bind(new IPEndPoint(IPAddress.Any, statusPort));
        _udp.Client.ReceiveTimeout = 500;

        _thread = new Thread(ReceiveLoop) { IsBackground = true };
        _thread.Start();
    }

    void Update()
    {
        lock (_lock)
        {
            if (!_dirty) return;
            _dirty = false;

            if (qualitySlider) qualitySlider.value = _pendingQuality / 100f;
            if (qualityLabel) qualityLabel.text = $"Learning: {_pendingQuality:0}%";
            if (statusLabel) statusLabel.text = _pendingStatus;
        }
    }

    void ReceiveLoop()
    {
        var remote = new IPEndPoint(IPAddress.Any, 0);
        while (!_quit)
        {
            try
            {
                byte[] data = _udp.Receive(ref remote);
                string msg = Encoding.ASCII.GetString(data).Trim();

                lock (_lock)
                {
                    if (msg.StartsWith("STATUS:QUALITY:"))
                    {
                        if (float.TryParse(msg.Substring(15), out float q))
                            _pendingQuality = q;
                        _pendingStatus = "TRAINING";
                    }
                    else if (msg.StartsWith("STATUS:"))
                    {
                        _pendingStatus = msg.Substring(7);
                    }
                    _dirty = true;
                }
            }
            catch (SocketException) { }
            catch (ObjectDisposedException) { break; }
        }
    }

    void OnDestroy()
    {
        _quit = true;
        _udp?.Close();
        _thread?.Join(300);
    }
}