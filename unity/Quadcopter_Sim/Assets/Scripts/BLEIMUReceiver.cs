// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL
// https://github.com/hannesgook/pidraqrl

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using UnityEngine;

public class BLEIMUReceiver : MonoBehaviour
{
    [Header("Mode")]
    public bool udpRelayMode = false;

    [Header("UDP Relay (used when udpRelayMode = true)")]
    public int udpPort = 5005;

    [Header("BLE Configuration (used when udpRelayMode = false)")]
    public string deviceName = "ESP32-BLE-FAST";
    public string txCharacteristic = "12345678-1234-5678-1234-56789abcdef1";
    public float scanTimeout = 10f;
    public float reconnectDelay = 2f;

    [Header("Latest Telemetry")]
    public double timestamp;
    public float roll;
    public float pitch;
    public float yaw;
    public float targetRollAngle;

    [Header("Gyro Rates")]
    public float gx;
    public float gy;
    public float gz;

    [Header("Stats")]
    public double secondsSinceLastPacket;
    public int packetCount;
    public float currentHz;
    public float targetHz = 100f; // what we expect from the ESP32
    public float hzEfficiency; // currentHz / targetHz as 0–100 %
    public int packetsDropped; // cumulative packets missed vs target
    public string connectionStatus = "Disconnected";

    // IMU packet (little-endian):
    //   marker(1) roll(4) pitch(4) yaw(4) gx(4) gy(4) gz(4) targetRoll(4) esp_ts(4) = 33 bytes
    const int IMU_SIZE = 33;
    const byte IMU_MARKER = (byte)'I';

    // Thread-safe internals
    readonly object stateLock = new object();
    readonly object queueLock = new object();
    readonly Queue<byte[]> packetQueue = new Queue<byte[]>();

    double pendingTimestamp;
    float pendingRoll, pendingPitch, pendingYaw;
    float pendingGx, pendingGy, pendingGz;
    long pendingLastTicks;
    int pendingPacketCount;
    float pendingTargetRollAngle;

    int rxCount;
    double hzTimerStart;

    // Rolling average Hz over last 16 inter-packet intervals - updates every packet
    const int HZ_WINDOW = 16;
    double[] _ipiBuffer = new double[16];
    int _ipiIndex = 0;
    int _ipiCount = 0;
    double _lastPacketTime = 0;
    public float instantHz; // updated at packet rate (~100 Hz), read by UI

    static readonly double TicksToSec = 1.0 / System.Diagnostics.Stopwatch.Frequency;

    // Delegates must be kept alive as fields so the GC doesn't collect them
    // while the native DLL holds a function pointer to them.
    ESP32BLE.NotifyCallback _notifyDelegate;
    ESP32BLE.DisconnectCallback _disconnectDelegate;

    bool appQuitting;
    bool deviceDisconnected; // set from native callback, read on main thread

    // UDP relay internals
    System.Net.Sockets.UdpClient _udpClient;
    Thread _udpThread;
    const int UDP_PKT_SIZE = 36; // double + 6 floats

    void Start()
    {
        hzTimerStart = NowSec();
        if (udpRelayMode)
        {
            connectionStatus = "UDP mode - waiting for Python...";
            Debug.Log($"[BLEIMUReceiver] UDP relay mode - listening on port {udpPort}");
            _udpClient = new System.Net.Sockets.UdpClient(udpPort);
            _udpClient.Client.ReceiveTimeout = 500;
            _udpThread = new Thread(UDPReceiveLoop) { IsBackground = true };
            _udpThread.Start();
        }
        else
        {
            StartCoroutine(BLELoop());
        }
        Debug.Log("[BLEIMUReceiver] Started.");
    }

    void Update()
    {
        // Print any DLL scan log entries - helps debug what BLE devices are seen
        DrainDLLLog();
        // Drain packet queue
        while (true)
        {
            byte[] pkt;
            lock (queueLock)
            {
                if (packetQueue.Count == 0) break;
                pkt = packetQueue.Dequeue();
            }
            ParseAndStore(pkt);
        }

        // Flush pending state to public fields
        lock (stateLock)
        {
            timestamp = pendingTimestamp;
            roll = pendingRoll;
            pitch = pendingPitch;
            yaw = pendingYaw;
            gx = pendingGx;
            gy = pendingGy;
            gz = pendingGz;
            targetRollAngle = pendingTargetRollAngle;
            packetCount = pendingPacketCount;

            if (pendingLastTicks > 0)
            {
                long now = System.Diagnostics.Stopwatch.GetTimestamp();
                secondsSinceLastPacket = (now - pendingLastTicks) * TicksToSec;
            }
        }

        // Hz meter - updates every second
        double nowSec = NowSec();
        double elapsed = nowSec - hzTimerStart;
        if (elapsed >= 1.0)
        {
            currentHz = (float)(rxCount / elapsed);
            hzEfficiency = targetHz > 0 ? Mathf.Clamp01(currentHz / targetHz) * 100f : 0f;
            int expected = Mathf.RoundToInt(targetHz * (float)elapsed);
            packetsDropped += Mathf.Max(0, expected - rxCount);
            rxCount = 0;
            hzTimerStart = nowSec;
            Debug.Log($"[BLEIMUReceiver] RX: {currentHz:F1} Hz  ({hzEfficiency:F0}% of {targetHz} Hz target)  dropped={packetsDropped}");
        }
    }

    void OnDestroy()
    {
        appQuitting = true;
        if (udpRelayMode)
        {
            _udpClient?.Close();
            _udpThread?.Join(300);
        }
        else
        {
            ESP32BLE.Disconnect();
        }
    }

    void OnApplicationQuit() => OnDestroy();

    IEnumerator BLELoop()
    {
        while (!appQuitting)
        {
            Debug.Log($"[BLEIMUReceiver] Scanning for '{deviceName}'...");
            connectionStatus = "Scanning...";
            ESP32BLE.StartScan();

            string foundAddress = null;
            float t0 = Time.unscaledTime;

            while (Time.unscaledTime - t0 < scanTimeout)
            {
                foundAddress = ESP32BLE.FindDeviceByName(deviceName);
                if (foundAddress != null) break;
                yield return null;
            }

            ESP32BLE.StopScan();

            if (foundAddress == null)
            {
                Debug.LogWarning("[BLEIMUReceiver] ESP32 not found, retrying...");
                yield return new WaitForSecondsRealtime(reconnectDelay);
                continue;
            }

            Debug.Log($"[BLEIMUReceiver] Found device, connecting...");

            deviceDisconnected = false;
            bool connectDone = false;
            bool connectOk = false;

            _disconnectDelegate = () => { deviceDisconnected = true; };

            _notifyDelegate = (ptr, len) =>
            {
                byte[] copy = new byte[len];
                Marshal.Copy(ptr, copy, 0, len);
                lock (queueLock) packetQueue.Enqueue(copy);
            };

            string capAddress = foundAddress;
            ThreadPool.QueueUserWorkItem(_ =>
            {
                bool ok = ESP32BLE.Connect(capAddress, _disconnectDelegate);
                if (ok) ok = ESP32BLE.Subscribe(txCharacteristic, _notifyDelegate);
                connectOk = ok;
                connectDone = true;
            });

            // Wait for thread pool work to finish (max 10s)
            t0 = Time.unscaledTime;
            while (!connectDone && Time.unscaledTime - t0 < 10f)
                yield return null;

            if (!connectOk)
            {
                Debug.LogWarning("[BLEIMUReceiver] Connect/Subscribe failed.");
                ESP32BLE.Disconnect();
                yield return new WaitForSecondsRealtime(reconnectDelay);
                continue;
            }

            Debug.Log("[BLEIMUReceiver] Connected and subscribed. Receiving IMU data.");
            connectionStatus = "Connected";

            while (!deviceDisconnected && !appQuitting)
                yield return null;

            Debug.Log($"[BLEIMUReceiver] Disconnected. Reconnecting in {reconnectDelay}s...");
            connectionStatus = "Reconnecting...";
            ESP32BLE.Disconnect();
            yield return new WaitForSecondsRealtime(reconnectDelay);
        }
    }

    void ParseAndStore(byte[] data)
    {
        if (data == null || data.Length < IMU_SIZE) return;
        if (data[0] != IMU_MARKER) return;

        int o = 1;
        float r = ReadF32(data, o); o += 4;
        float p = ReadF32(data, o); o += 4;
        float yv = ReadF32(data, o); o += 4;
        float gxv = ReadF32(data, o); o += 4;
        float gyv = ReadF32(data, o); o += 4;
        float gzv = ReadF32(data, o); o += 4;
        float targetRv = ReadF32(data, o);

        double now = NowSec();
        rxCount++;

        if (_lastPacketTime > 0)
        {
            double ipi = now - _lastPacketTime;
            if (ipi > 0 && ipi < 1.0)
            {
                _ipiBuffer[_ipiIndex % HZ_WINDOW] = ipi;
                _ipiIndex++;
                if (_ipiCount < HZ_WINDOW) _ipiCount++;
                double sum = 0;
                for (int i = 0; i < _ipiCount; i++) sum += _ipiBuffer[i];
                instantHz = (float)(_ipiCount / sum);
            }
        }
        _lastPacketTime = now;

        lock (stateLock)
        {
            pendingTimestamp = now;
            pendingRoll = r;
            pendingPitch = p;
            pendingYaw = yv;
            pendingGx = gxv;
            pendingGy = gyv;
            pendingGz = gzv;
            pendingTargetRollAngle = targetRv;
            pendingLastTicks = System.Diagnostics.Stopwatch.GetTimestamp();
            pendingPacketCount++;
        }
    }

    static float ReadF32(byte[] b, int o)
    {
        if (BitConverter.IsLittleEndian) return BitConverter.ToSingle(b, o);
        byte[] t = new byte[4];
        Buffer.BlockCopy(b, o, t, 0, 4);
        Array.Reverse(t);
        return BitConverter.ToSingle(t, 0);
    }

    static double NowSec()
        => System.Diagnostics.Stopwatch.GetTimestamp() * TicksToSec;

    void UDPReceiveLoop()
    {
        var remote = new System.Net.IPEndPoint(System.Net.IPAddress.Any, 0);
        while (!appQuitting)
        {
            try
            {
                byte[] data = _udpClient.Receive(ref remote);
                if (data == null || data.Length < UDP_PKT_SIZE) continue;

                double ts = BitConverter.ToDouble(data, 0);
                float r = BitConverter.ToSingle(data, 8);
                float p = BitConverter.ToSingle(data, 12);
                float yv = BitConverter.ToSingle(data, 16);
                float gxv = BitConverter.ToSingle(data, 20);
                float gyv = BitConverter.ToSingle(data, 24);
                float gzv = BitConverter.ToSingle(data, 28);
                float targetRv = BitConverter.ToSingle(data, 32);

                UnityEngine.Debug.Log($"[UDP] r={r:F2} p={p:F2} y={yv:F2} targetR={targetRv:F2}");

                double now = NowSec();
                rxCount++;

                if (_lastPacketTime > 0)
                {
                    double ipi = now - _lastPacketTime;
                    if (ipi > 0 && ipi < 1.0)
                    {
                        _ipiBuffer[_ipiIndex % HZ_WINDOW] = ipi;
                        _ipiIndex++;
                        if (_ipiCount < HZ_WINDOW) _ipiCount++;
                        double sum = 0;
                        for (int i = 0; i < _ipiCount; i++) sum += _ipiBuffer[i];
                        instantHz = (float)(_ipiCount / sum);
                    }
                }
                _lastPacketTime = now;

                lock (stateLock)
                {
                    pendingTimestamp = now;
                    pendingRoll = r;
                    pendingPitch = p;
                    pendingYaw = yv;
                    pendingGx = gxv;
                    pendingGy = gyv;
                    pendingGz = gzv;
                    pendingTargetRollAngle = targetRv;
                    pendingLastTicks = System.Diagnostics.Stopwatch.GetTimestamp();
                    pendingPacketCount++;
                }

                if (connectionStatus != "Connected (UDP)")
                {
                    connectionStatus = "Connected (UDP)";
                }
            }
            catch (System.Net.Sockets.SocketException) { }
            catch (ObjectDisposedException) { break; }
        }
    }

    void DrainDLLLog()
    {
        var buf = new StringBuilder(256);
        while (ESP32BLE.PopLog(buf))
            Debug.Log("[ESP32BLE.dll] " + buf);
    }
}

internal static class ESP32BLE
{
    const string DLL = "ESP32BLE";

    // Delegate types - kept alive as fields in BLEIMUReceiver above
    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void NotifyCallback(IntPtr data, int length);

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void DisconnectCallback();

    // scan
    [DllImport(DLL, CallingConvention = CallingConvention.StdCall)]
    static extern void BLE_StartScan();

    [DllImport(DLL, CallingConvention = CallingConvention.StdCall)]
    static extern void BLE_StopScan();

    [DllImport(DLL, CallingConvention = CallingConvention.StdCall)]
    static extern int BLE_GetDeviceCount();

    [DllImport(DLL, CallingConvention = CallingConvention.StdCall, CharSet = CharSet.Ansi)]
    static extern bool BLE_GetDeviceName(int index,
        [Out] StringBuilder buf, int bufLen);

    [DllImport(DLL, CallingConvention = CallingConvention.StdCall, CharSet = CharSet.Ansi)]
    static extern bool BLE_GetDeviceAddress(int index,
        [Out] StringBuilder buf, int bufLen);

    // connect
    [DllImport(DLL, CallingConvention = CallingConvention.StdCall, CharSet = CharSet.Ansi)]
    static extern bool BLE_Connect(
        [MarshalAs(UnmanagedType.LPStr)] string addressHex,
        DisconnectCallback onDisconnect);

    [DllImport(DLL, CallingConvention = CallingConvention.StdCall)]
    static extern bool BLE_IsConnected();

    // Subscribe
    [DllImport(DLL, CallingConvention = CallingConvention.StdCall, CharSet = CharSet.Ansi)]
    static extern bool BLE_Subscribe(
        [MarshalAs(UnmanagedType.LPStr)] string charUuid,
        NotifyCallback onNotify);

    // disconnect
    [DllImport(DLL, CallingConvention = CallingConvention.StdCall)]
    static extern void BLE_Disconnect();


    public static void StartScan() => BLE_StartScan();
    public static void StopScan() => BLE_StopScan();
    public static bool IsConnected => BLE_IsConnected();

    public static string FindDeviceByName(string nameFilter)
    {
        int count = BLE_GetDeviceCount();
        var nameBuf = new StringBuilder(256);
        var addrBuf = new StringBuilder(32);

        for (int i = 0; i < count; i++)
        {
            nameBuf.Clear(); addrBuf.Clear();
            BLE_GetDeviceName(i, nameBuf, nameBuf.Capacity);
            if (!nameBuf.ToString().Contains(nameFilter)) continue;
            BLE_GetDeviceAddress(i, addrBuf, addrBuf.Capacity);
            return addrBuf.ToString();
        }
        return null;
    }

    public static bool Connect(string addressHex, DisconnectCallback onDisconnect)
        => BLE_Connect(addressHex, onDisconnect);

    public static bool Subscribe(string charUuid, NotifyCallback onNotify)
        => BLE_Subscribe(charUuid, onNotify);

    public static void Disconnect() { try { BLE_Disconnect(); } catch { } }

    public static bool PopLog(StringBuilder buf)
    {
        buf.Clear();
        return BLE_PopLog(buf, buf.Capacity);
    }

    public static int WatcherStatus => BLE_GetWatcherStatus();

    [DllImport(DLL, CallingConvention = CallingConvention.StdCall, CharSet = CharSet.Ansi)]
    static extern bool BLE_PopLog([Out] StringBuilder buf, int bufLen);

    [DllImport(DLL, CallingConvention = CallingConvention.StdCall)]
    static extern int BLE_GetWatcherStatus();
}