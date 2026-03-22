// Copyright (c) 2025-2026 Hannes Göök
// MIT License - PidraQRL Project
// https://github.com/hannesgook/pqrl2

import 'dart:async';
import 'dart:convert';
import 'dart:math';
import 'dart:ui'; // for ImageFilter
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:permission_handler/permission_handler.dart';
import 'dart:typed_data';
import 'package:sensors_plus/sensors_plus.dart';

void main() {
  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ),
  );
  runApp(const DroneApp());
}

class DroneApp extends StatelessWidget {
  const DroneApp({super.key});

  @override
  Widget build(BuildContext context) {
    const bg = Color(0xFF000000);
    const primary = Colors.white;
    const surface = Color(0xFF121212);

    final scheme = ColorScheme.dark(
      primary: primary,
      secondary: primary.withOpacity(0.8),
      surface: surface,
      background: bg,
      onBackground: Colors.white,
      onSurface: Colors.white,
    );

    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'PidraQ',
      theme: ThemeData(
        useMaterial3: true,
        brightness: Brightness.dark,
        colorScheme: scheme,
        scaffoldBackgroundColor: bg,
        fontFamily: 'RobotoMono',
        sliderTheme: SliderThemeData(
          activeTrackColor: Colors.white,
          inactiveTrackColor: Colors.white24,
          thumbColor: Colors.white,
          trackHeight: 2,
          overlayColor: Colors.white.withOpacity(0.1),
          thumbShape: const RoundSliderThumbShape(
            enabledThumbRadius: 6,
            elevation: 2,
          ),
        ),
        textTheme: const TextTheme(
          displaySmall: TextStyle(
            color: Colors.white,
            fontWeight: FontWeight.w300,
            letterSpacing: -0.5,
          ),
          bodyLarge: TextStyle(color: Colors.white70),
          bodyMedium: TextStyle(color: Colors.white54),
        ),
      ),
      home: const ControlScreen(),
    );
  }
}

class GlassContainer extends StatelessWidget {
  final Widget child;
  final EdgeInsetsGeometry? padding;
  final double? width;
  final double? height;
  final VoidCallback? onTap;

  const GlassContainer({
    super.key,
    required this.child,
    this.padding,
    this.width,
    this.height,
    this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(16),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
        child: GestureDetector(
          onTap: onTap,
          child: Container(
            width: width,
            height: height,
            padding: padding ?? const EdgeInsets.all(16),
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.08),
              borderRadius: BorderRadius.circular(16),
              border: Border.all(
                color: Colors.white.withOpacity(0.2),
                width: 1,
              ),
            ),
            child: child,
          ),
        ),
      ),
    );
  }
}

enum RollSequenceType { sine, square, triangle, chirp }

extension RollSequenceTypeLabel on RollSequenceType {
  String get label {
    switch (this) {
      case RollSequenceType.sine:
        return 'SINE';
      case RollSequenceType.square:
        return 'SQUARE';
      case RollSequenceType.triangle:
        return 'TRI';
      case RollSequenceType.chirp:
        return 'CHIRP';
    }
  }

  String get description {
    switch (this) {
      case RollSequenceType.sine:
        return 'Smooth sinusoidal oscillation';
      case RollSequenceType.square:
        return 'Abrupt step between +-A';
      case RollSequenceType.triangle:
        return 'Linear ramp between +-A';
      case RollSequenceType.chirp:
        return 'Sine with increasing frequency';
    }
  }
}

double evaluateRollSequence({
  required RollSequenceType type,
  required double t,
  required double amplitude, // 0 to 1
  required double frequency, // Hz  (for chirp: starting freq)
  double chirpEndFreq = 3.0, // Hz  (chirp target freq)
  double chirpDuration = 10.0, // seconds over which freq sweeps
}) {
  switch (type) {
    case RollSequenceType.sine:
      return amplitude * sin(2 * pi * frequency * t);

    case RollSequenceType.square:
      final phase = (t * frequency) % 1.0;
      return amplitude * (phase < 0.5 ? 1.0 : -1.0);

    case RollSequenceType.triangle:
      final phase = (t * frequency) % 1.0;
      final v = phase < 0.5 ? 1.0 - 4.0 * phase : -1.0 + 4.0 * (phase - 0.5);
      return amplitude * v;

    case RollSequenceType.chirp:
      final tClamped = t.clamp(0.0, chirpDuration);
      final k = (chirpEndFreq - frequency) / chirpDuration;
      final phase =
          2 * pi * (frequency * tClamped + 0.5 * k * tClamped * tClamped);
      return amplitude * sin(phase);
  }
}

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});
  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  FlutterReactiveBle? _ble;

  Characteristic? _txResolved;
  StreamSubscription<List<int>>? _notifySub;

  bool _tickInFlight = false;

  final GlobalKey<_FixedBaseJoystickState> _tiltJoyKey =
      GlobalKey<_FixedBaseJoystickState>();

  // Accelerometer tilt mode
  bool _tiltModeAccel = false;
  StreamSubscription<AccelerometerEvent>? _accelSub;

  double _pActual = 0.0;
  double _iActual = 0.0;
  double _dActual = 0.0;

  double _p2Actual = 0.0;
  double _i2Actual = 0.0;
  double _d2Actual = 0.0;

  double _angleKPActual = 0.0;

  double get P => _pActual;
  double get I => _iActual;
  double get D => _dActual;

  double get P2 => _p2Actual;
  double get I2 => _i2Actual;
  double get D2 => _d2Actual;

  double _defP = 0.37;
  double _defI = 0.0;
  double _defD = 0.06;

  double _defP2 = 0.0;
  double _defI2 = 0.0;
  double _defD2 = 0.0;

  double _defAngleKP = 3.3;
  static const double _defaultAngleKPMult = 10.0;
  double _angleKPMult = _defaultAngleKPMult;

  bool _pidOuterLocked = false;

  static const double _defaultPMult = 1.0;
  static const double _defaultIMult = 0.01;
  static const double _defaultDMult = 0.1;
  static const double _defaultP2Mult = 1.0;
  static const double _defaultI2Mult = 1.0;
  static const double _defaultD2Mult = 1.0;

  double _pMult = _defaultPMult;
  double _iMult = _defaultIMult;
  double _dMult = _defaultDMult;
  double _p2Mult = _defaultP2Mult;
  double _i2Mult = _defaultI2Mult;
  double _d2Mult = _defaultD2Mult;

  double _throttle = 0.0;
  double get throttle => _throttle;

  bool _joystickActive = false;
  bool _writeBusy = false;
  Uint8List? _pendingBytes;

  Timer? _sendTimer;
  final Duration _sendInterval = const Duration(milliseconds: 5);
  double _latestX = 0, _latestY = 0;

  final Uuid _serviceUuid = Uuid.parse("6e400001-b5a3-f393-e0a9-e50e24dcca9F");
  final Uuid _rxCharUuid = Uuid.parse("6e400002-b5a3-f393-e0a9-e50e24dcca9F");
  final Uuid _txCharUuid = Uuid.parse("6e400003-b5a3-f393-e0a9-e50e24dcca9F");

  final String _targetName = "BLE-LoRa-Bridge";

  DiscoveredDevice? _device;
  StreamSubscription<DiscoveredDevice>? _scanSub;
  StreamSubscription<ConnectionStateUpdate>? _connSub;

  Characteristic? _rxResolved;

  bool _scanning = false;
  bool _connecting = false;
  bool _connected = false;
  bool _ready = false;

  String _status = "DISCONNECTED";
  String _lastError = "";

  int _diagServiceCount = 0;
  bool _diagHasService = false;
  bool _diagHasTx = false;
  bool _diagRxWritable = false;

  double _jx = 0;
  double _jy = 0;
  double _jr = 0;
  double _latestR = 0;
  bool _tiltLocked = false;
  bool _yawLocked = false;

  bool _pidPitchLocked = false;
  bool _pidYawLocked = false;

  final ValueNotifier<double> _yawAngleNotifier = ValueNotifier(0.0);

  // scroll lock
  int _joystickTouchCount = 0;
  final ScrollController _joystickScrollController = ScrollController();

  // roll sequence
  RollSequenceType _selectedSequence = RollSequenceType.sine;
  bool _sequencePlaying = false;
  double _seqAmplitude = 0.3;
  double _seqFrequency = 0.5; // Hz
  double _seqChirpEndFreq = 3.0;
  double _seqChirpDuration = 10.0;
  Timer? _seqTimer;
  final Stopwatch _seqStopwatch = Stopwatch();

  void _startSequence() {
    if (_tiltLocked) return;
    if (_sequencePlaying) return;

    _seqStopwatch.reset();
    _seqStopwatch.start();
    setState(() => _sequencePlaying = true);

    _seqTimer = Timer.periodic(const Duration(milliseconds: 16), (_) {
      if (!mounted) return;
      final t = _seqStopwatch.elapsedMilliseconds / 1000.0;
      final rollVal = evaluateRollSequence(
        type: _selectedSequence,
        t: t,
        amplitude: _seqAmplitude,
        frequency: _seqFrequency,
        chirpEndFreq: _seqChirpEndFreq,
        chirpDuration: _seqChirpDuration,
      );

      _latestX = rollVal.clamp(-1.0, 1.0);
      _latestY = 0.0; // only roll for now

      if ((rollVal - _jx).abs() > 0.01) {
        setState(() {
          _jx = _latestX;
          _jy = 0.0;
        });
      }
    });
  }

  void _stopSequence({bool resetStick = false}) {
    _seqTimer?.cancel();
    _seqTimer = null;
    _seqStopwatch.stop();

    if (resetStick) {
      _latestX = 0;
      _latestY = 0;
      setState(() {
        _sequencePlaying = false;
        _jx = 0;
        _jy = 0;
      });
    } else {
      final snapX = _latestX;
      final snapY = _latestY;
      _tiltJoyKey.currentState?.syncFromExternal(snapX, snapY);
      setState(() => _sequencePlaying = false);
    }
  }

  @override
  void initState() {
    super.initState();

    _pActual = _defP;
    _iActual = _defI;
    _dActual = _defD;
    _p2Actual = _defP2;
    _i2Actual = _defI2;
    _d2Actual = _defD2;
    _angleKPActual = _defAngleKP;

    _ble = FlutterReactiveBle();
    _ble!.statusStream.listen((s) {
      if (s != BleStatus.ready) {
        setState(() => _status = "BLUETOOTH NOT READY");
      }
    });

    _ensurePermissions();
  }

  double _accelBaseX = 0.0;
  double _accelBaseY = 0.0;
  bool _accelCalibrated = false;

  Orientation _screenOrientation = Orientation.portrait;

  void _setTiltMode(bool useAccel) {
    if (_tiltLocked) return;

    if (!useAccel) {
      _accelSub?.cancel();
      _accelSub = null;
      _accelCalibrated = false;
      _latestX = 0;
      _latestY = 0;
      setState(() {
        _tiltModeAccel = false;
        _jx = 0;
        _jy = 0;
      });
      return;
    }

    _accelCalibrated = false;
    setState(() => _tiltModeAccel = true);

    _accelSub?.cancel();
    _accelSub =
        accelerometerEventStream(
          samplingPeriod: const Duration(milliseconds: 20),
        ).listen((event) {
          const g = 9.8;

          double rawX, rawY;
          switch (_screenOrientation) {
            case Orientation.portrait:
              rawX = -event.x / g;
              rawY = event.y / g;
              break;
            case Orientation.landscape:
              if (event.x >= 0) {
                rawX = event.y / g;
                rawY = event.x / g;
              } else {
                rawX = -event.y / g;
                rawY = -event.x / g;
              }
              break;
          }

          if (!_accelCalibrated) {
            _accelBaseX = rawX;
            _accelBaseY = rawY;
            _accelCalibrated = true;
            return;
          }

          final x = (rawX - _accelBaseX).clamp(-1.0, 1.0);
          final y = (rawY - _accelBaseY).clamp(-1.0, 1.0);

          _latestX = x;
          _latestY = y;

          if ((x - _jx).abs() > 0.01 || (y - _jy).abs() > 0.01) {
            if (mounted) {
              setState(() {
                _jx = x;
                _jy = y;
              });
            }
          }
        });
  }

  Widget _buildModeChip(String label, bool selected, VoidCallback onTap) {
    return GestureDetector(
      onTap: onTap,
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 180),
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 5),
        decoration: BoxDecoration(
          color: selected ? Colors.white : Colors.white10,
          borderRadius: BorderRadius.circular(20),
          border: Border.all(
            color: selected ? Colors.white : Colors.white24,
            width: 1,
          ),
        ),
        child: Text(
          label,
          style: TextStyle(
            fontSize: 9,
            fontWeight: FontWeight.bold,
            letterSpacing: 1.2,
            color: selected ? Colors.black : Colors.white54,
          ),
        ),
      ),
    );
  }

  void _startNotify() {
    if (_device == null || _txResolved == null || _ble == null) return;
    _notifySub?.cancel();

    final qc = QualifiedCharacteristic(
      deviceId: _device!.id,
      serviceId: _serviceUuid,
      characteristicId: _txCharUuid,
    );

    _notifySub = _ble!
        .subscribeToCharacteristic(qc)
        .listen((data) {}, onError: (e) {});
  }

  @override
  void dispose() {
    _seqTimer?.cancel();
    _accelSub?.cancel();
    _scanSub?.cancel();
    _connSub?.cancel();
    _notifySub?.cancel();
    _joystickScrollController.dispose();
    _yawAngleNotifier.dispose();
    super.dispose();
  }

  Future<void> _ensurePermissions() async {
    final scan = await Permission.bluetoothScan.request();
    final conn = await Permission.bluetoothConnect.request();
    final loc = await Permission.locationWhenInUse.request();

    if (!scan.isGranted || !conn.isGranted || !loc.isGranted) {
      setState(() => _status = "PERMISSIONS REQUIRED");
    }
  }

  Future<void> _scanAndConnect() async {
    if (_scanning || _connecting || _connected) return;

    if (_ble == null) {
      setState(() => _status = "NO BLE");
      return;
    }

    setState(() {
      _status = "SCANNING...";
      _scanning = true;
      _ready = false;
      _lastError = "";
      _diagServiceCount = 0;
      _diagHasService = false;
      _diagHasTx = false;
      _diagRxWritable = false;
      _rxResolved = null;
    });

    _scanSub = _ble!
        .scanForDevices(
          withServices: [_serviceUuid],
          scanMode: ScanMode.lowLatency,
        )
        .listen(
          (d) {
            if (d.name == _targetName ||
                d.serviceUuids.contains(_serviceUuid)) {
              _device = d;
              _scanSub?.cancel();
              _scanning = false;
              _connect();
            }
          },
          onError: (e) {
            setState(() {
              _status = "SCAN ERROR";
              _lastError = e.toString();
              _scanning = false;
            });
          },
        );

    await Future.delayed(const Duration(seconds: 8));
    if (mounted && !_connected && _scanning) {
      _scanSub?.cancel();
      setState(() {
        _status = "NOT FOUND";
        _scanning = false;
      });
    }
  }

  Future<void> _connect() async {
    if (_device == null || _ble == null) return;
    setState(() {
      _connecting = true;
      _status = "CONNECTING...";
      _ready = false;
      _lastError = "";
    });

    _connSub = _ble!
        .connectToDevice(
          id: _device!.id,
          connectionTimeout: const Duration(seconds: 10),
        )
        .listen(
          (update) async {
            if (update.connectionState == DeviceConnectionState.connected) {
              _connected = true;
              _connecting = false;
              _status = "OPTIMIZING...";
              try {
                await _ble!.requestMtu(deviceId: _device!.id, mtu: 185);
              } catch (_) {}

              _status = "DISCOVERING...";
              final ok = await _resolveExactInstances();
              if (!ok) {
                _disconnect(msg: "BAD SERVICE");
                return;
              }
              _ready = true;
              setState(() => _status = "CONNECTED");
              _startNotify();
              _startSending();
            } else if (update.connectionState ==
                DeviceConnectionState.disconnected) {
              _disconnect(msg: "DISCONNECTED");
            }
            setState(() {});
          },
          onError: (e) {
            _disconnect(msg: "ERROR");
            _lastError = e.toString();
          },
        );
  }

  Future<bool> _resolveExactInstances() async {
    if (_ble == null) return false;
    try {
      final services = await _ble!.getDiscoveredServices(_device!.id);
      _diagServiceCount = services.length;

      final nusServices = services.where((s) => s.id == _serviceUuid).toList();
      _diagHasService = nusServices.isNotEmpty;

      Service? chosen;
      Characteristic? rxC;
      Characteristic? txC;

      for (final s in nusServices) {
        final rxList = s.characteristics
            .where((c) => c.id == _rxCharUuid)
            .toList();
        final txList = s.characteristics
            .where((c) => c.id == _txCharUuid)
            .toList();
        if (rxList.isNotEmpty && txList.isNotEmpty) {
          final rxWritable = rxList.firstWhere(
            (c) => c.isWritableWithoutResponse || c.isWritableWithResponse,
            orElse: () => rxList.first,
          );
          chosen = s;
          rxC = rxWritable;
          txC = txList.first;
          break;
        }
      }

      if (chosen == null || rxC == null || txC == null) {
        _diagHasTx = false;
        _diagRxWritable = false;
        _rxResolved = null;
        _txResolved = null;
        setState(() {});
        return false;
      }

      _rxResolved = rxC;
      _txResolved = txC;
      _diagHasTx = true;
      _diagRxWritable =
          rxC.isWritableWithoutResponse || rxC.isWritableWithResponse;

      setState(() {});
      return _diagRxWritable;
    } catch (e) {
      _lastError = "resolve: $e";
      setState(() {});
      return false;
    }
  }

  void _disconnect({String msg = "DISCONNECTED"}) {
    _connSub?.cancel();
    _notifySub?.cancel();
    _stopSending();
    _connected = false;
    _connecting = false;
    _scanning = false;
    _ready = false;
    _rxResolved = null;
    _txResolved = null;
    setState(() {
      _status = msg;
      _lastError = "";
    });
  }

  void _startSending() {
    _sendTimer ??= Timer.periodic(_sendInterval, (_) => _tickSend());
  }

  void _stopSending() {
    _sendTimer?.cancel();
    _sendTimer = null;
  }

  int _map11(double v) => ((v + 1.0) * 127.5).clamp(0, 255).round();
  int _map01(double v) => (v * 255).clamp(0, 255).round();

  Future<void> _tickSend() async {
    if (_tickInFlight) return;
    _tickInFlight = true;

    try {
      if (!_ready || !_connected || _rxResolved == null) return;

      final t = _map01(throttle);
      final x = _map11(_latestX);
      final y = _map11(_latestY);
      final r = _map11(_latestR);

      final bd = ByteData(34);
      bd.setUint8(0, 0x53);
      bd.setUint8(1, t);
      bd.setUint8(2, x);
      bd.setUint8(3, y);
      bd.setUint8(4, r);
      bd.setFloat32(5, P, Endian.little);
      bd.setFloat32(9, I, Endian.little);
      bd.setFloat32(13, D, Endian.little);
      bd.setFloat32(17, P2, Endian.little);
      bd.setFloat32(21, I2, Endian.little);
      bd.setFloat32(25, D2, Endian.little);
      bd.setFloat32(29, _angleKPActual, Endian.little);

      final bytes = bd.buffer.asUint8List();
      int chk = 0;
      for (int i = 1; i < 33; i++) chk ^= bytes[i];
      bd.setUint8(33, chk);

      await _writeCoalesced(bd.buffer.asUint8List());
    } finally {
      _tickInFlight = false;
    }
  }

  Future<void> _writeCoalesced(Uint8List bytes) async {
    _pendingBytes = bytes;
    if (_writeBusy) return;
    if (_rxResolved == null) return;

    _writeBusy = true;
    try {
      while (_pendingBytes != null && _rxResolved != null && _connected) {
        final next = _pendingBytes!;
        _pendingBytes = null;
        await _rxResolved!.write(next, withResponse: true);
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _status = "WRITE FAIL";
          _lastError = e.toString();
        });
      }
    } finally {
      _writeBusy = false;
    }
  }

  @override
  Widget build(BuildContext context) {
    _screenOrientation = MediaQuery.of(context).orientation;

    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            children: [
              _buildHeader(),
              const SizedBox(height: 16),
              Expanded(
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    _buildThrottleColumn(),
                    const SizedBox(width: 16),
                    Expanded(child: _buildCenterPanel()),
                    const SizedBox(width: 16),
                    _buildJoystickColumn(),
                  ],
                ),
              ),
              if (_lastError.isNotEmpty)
                Padding(
                  padding: const EdgeInsets.only(top: 8.0),
                  child: Text(
                    _lastError,
                    style: const TextStyle(
                      color: Colors.redAccent,
                      fontSize: 10,
                    ),
                  ),
                ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildHeader() {
    return GlassContainer(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Row(
            children: [
              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    "PidraQ",
                    style: Theme.of(context).textTheme.titleMedium?.copyWith(
                      letterSpacing: 2.0,
                      fontWeight: FontWeight.bold,
                      height: 1.0,
                    ),
                  ),
                  const SizedBox(height: 4),
                  const Text(
                    "by Hannes Göök - Est. 2025",
                    style: TextStyle(
                      fontSize: 10,
                      color: Colors.white54,
                      letterSpacing: 1.0,
                    ),
                  ),
                ],
              ),
            ],
          ),
          Row(
            children: [
              _buildStatusPill(_status),
              const SizedBox(width: 12),
              _buildGlassIconButton(
                icon: Icons.link,
                onTap: (!_scanning && !_connecting && !_connected)
                    ? _scanAndConnect
                    : null,
                active: _scanning || _connecting,
              ),
              const SizedBox(width: 8),
              _buildGlassIconButton(
                icon: Icons.link_off,
                onTap: _connected ? _disconnect : null,
                active: false,
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildStatusPill(String text) {
    Color dotColor;
    if (text == "CONNECTED") {
      dotColor = Colors.greenAccent;
    } else if (text.startsWith("SCAN")) {
      dotColor = Colors.yellowAccent;
    } else if (text.startsWith("CONNECTING")) {
      dotColor = Colors.cyanAccent;
    } else if (text.startsWith("DISC") ||
        text.startsWith("ERR") ||
        text.startsWith("NOT") ||
        text.startsWith("BAD") ||
        text.startsWith("WRITE")) {
      dotColor = Colors.redAccent;
    } else {
      dotColor = Colors.grey;
    }

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.white10,
        borderRadius: BorderRadius.circular(20),
        border: Border.all(
          color: text == "CONNECTED" || text == "DISCONNECTED"
              ? dotColor.withOpacity(0.5)
              : Colors.white12,
          width: 1,
        ),
      ),
      child: Row(
        children: [
          Container(
            width: 8,
            height: 8,
            decoration: BoxDecoration(color: dotColor, shape: BoxShape.circle),
          ),
          const SizedBox(width: 8),
          Text(text, style: const TextStyle(fontSize: 10, letterSpacing: 1.0)),
        ],
      ),
    );
  }

  Widget _buildGlassIconButton({
    required IconData icon,
    VoidCallback? onTap,
    bool active = false,
  }) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: active
              ? Colors.white.withOpacity(0.2)
              : Colors.white.withOpacity(0.05),
          shape: BoxShape.circle,
          border: Border.all(
            color: active ? Colors.white : Colors.white24,
            width: 1,
          ),
        ),
        child: Icon(
          icon,
          size: 18,
          color: onTap != null ? Colors.white : Colors.white24,
        ),
      ),
    );
  }

  Widget _buildThrottleColumn() {
    return GlassContainer(
      width: 100,
      child: Column(
        children: [
          const Text(
            "THR",
            style: TextStyle(
              fontSize: 12,
              letterSpacing: 1.5,
              color: Colors.white54,
              fontWeight: FontWeight.w300,
            ),
          ),
          const SizedBox(height: 8),
          Text(
            (throttle * 100).toInt().toString(),
            style: const TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
              color: Colors.white,
            ),
          ),
          const SizedBox(height: 16),
          Expanded(
            child: RotatedBox(
              quarterTurns: -1,
              child: SliderTheme(
                data: SliderTheme.of(context).copyWith(
                  trackHeight: 40,
                  activeTrackColor: Colors.white.withOpacity(0.15),
                  inactiveTrackColor: Colors.transparent,
                  thumbColor: Colors.transparent,
                  overlayShape: SliderComponentShape.noOverlay,
                  thumbShape: SliderComponentShape.noThumb,
                  trackShape: _GlassTrackShape(activeValue: throttle),
                ),
                child: Slider(
                  value: throttle,
                  min: 0.0,
                  max: 1.0,
                  onChanged: (v) {
                    setState(() => _throttle = v);
                    _startSending();
                  },
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  // roll Sequence
  Widget _buildSequencePanel() {
    final bool canStart = !_tiltLocked && !_tiltModeAccel;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            const Text(
              "ROLL SEQ",
              style: TextStyle(
                color: Colors.white,
                fontSize: 11,
                fontWeight: FontWeight.bold,
              ),
            ),
            // Play / Stop button
            GestureDetector(
              onTap: canStart
                  ? () {
                      if (_sequencePlaying) {
                        _stopSequence(resetStick: false);
                      } else {
                        _startSequence();
                      }
                    }
                  : null,
              child: AnimatedContainer(
                duration: const Duration(milliseconds: 180),
                padding: const EdgeInsets.symmetric(
                  horizontal: 10,
                  vertical: 5,
                ),
                decoration: BoxDecoration(
                  color: !canStart
                      ? Colors.white10
                      : _sequencePlaying
                      ? Colors.redAccent.withOpacity(0.25)
                      : Colors.white.withOpacity(0.12),
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(
                    color: !canStart
                        ? Colors.white12
                        : _sequencePlaying
                        ? Colors.redAccent.withOpacity(0.7)
                        : Colors.white38,
                    width: 1,
                  ),
                ),
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Icon(
                      _sequencePlaying ? Icons.stop : Icons.play_arrow,
                      size: 13,
                      color: !canStart
                          ? Colors.white24
                          : _sequencePlaying
                          ? Colors.redAccent
                          : Colors.white,
                    ),
                    const SizedBox(width: 4),
                    Text(
                      _sequencePlaying ? "STOP" : "RUN",
                      style: TextStyle(
                        fontSize: 9,
                        fontWeight: FontWeight.bold,
                        letterSpacing: 1.1,
                        color: !canStart
                            ? Colors.white24
                            : _sequencePlaying
                            ? Colors.redAccent
                            : Colors.white,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),

        if (!canStart && !_sequencePlaying)
          Padding(
            padding: const EdgeInsets.only(top: 4.0),
            child: Text(
              _tiltLocked ? "Unlock tilt to enable" : "Disable accel mode",
              style: const TextStyle(fontSize: 8, color: Colors.redAccent),
            ),
          ),

        const SizedBox(height: 10),

        // Waveform type selector
        SingleChildScrollView(
          scrollDirection: Axis.horizontal,
          child: Row(
            children: RollSequenceType.values.map((type) {
              final sel = _selectedSequence == type;
              return Padding(
                padding: const EdgeInsets.only(right: 6),
                child: GestureDetector(
                  onTap: _sequencePlaying
                      ? null
                      : () => setState(() => _selectedSequence = type),
                  child: AnimatedContainer(
                    duration: const Duration(milliseconds: 150),
                    padding: const EdgeInsets.symmetric(
                      horizontal: 9,
                      vertical: 4,
                    ),
                    decoration: BoxDecoration(
                      color: sel ? Colors.white : Colors.white10,
                      borderRadius: BorderRadius.circular(6),
                      border: Border.all(
                        color: sel ? Colors.white : Colors.white24,
                        width: 1,
                      ),
                    ),
                    child: Text(
                      type.label,
                      style: TextStyle(
                        fontSize: 9,
                        fontWeight: FontWeight.bold,
                        letterSpacing: 1.1,
                        color: sel ? Colors.black : Colors.white54,
                      ),
                    ),
                  ),
                ),
              );
            }).toList(),
          ),
        ),

        const SizedBox(height: 12),

        _buildSeqSliderRow(
          label: "AMP",
          value: _seqAmplitude,
          min: 0.05,
          max: 1.0,
          displayDecimals: 2,
          onChanged: (v) => setState(() => _seqAmplitude = v),
        ),

        const SizedBox(height: 6),

        _buildSeqSliderRow(
          label: "FREQ",
          value: _seqFrequency,
          min: 0.1,
          max: 2.0,
          displayDecimals: 2,
          unit: "Hz",
          onChanged: (v) => setState(() => _seqFrequency = v),
        ),

        if (_selectedSequence == RollSequenceType.chirp) ...[
          const SizedBox(height: 6),
          _buildSeqSliderRow(
            label: "F END",
            value: _seqChirpEndFreq,
            min: 0.1,
            max: 10.0,
            displayDecimals: 1,
            unit: "Hz",
            onChanged: _sequencePlaying
                ? null
                : (v) => setState(() => _seqChirpEndFreq = v),
          ),
          const SizedBox(height: 6),
          _buildSeqSliderRow(
            label: "DUR",
            value: _seqChirpDuration,
            min: 2.0,
            max: 30.0,
            displayDecimals: 0,
            unit: "s",
            onChanged: _sequencePlaying
                ? null
                : (v) => setState(() => _seqChirpDuration = v),
          ),
        ],
      ],
    );
  }

  Widget _buildSeqSliderRow({
    required String label,
    required double value,
    required double min,
    required double max,
    required int displayDecimals,
    String unit = "",
    required ValueChanged<double>? onChanged,
  }) {
    final disabled = onChanged == null;
    return IgnorePointer(
      ignoring: disabled,
      child: AnimatedOpacity(
        opacity: disabled ? 0.35 : 1.0,
        duration: const Duration(milliseconds: 200),
        child: Row(
          children: [
            SizedBox(
              width: 36,
              child: Text(
                label,
                style: const TextStyle(
                  fontSize: 8,
                  color: Colors.white54,
                  letterSpacing: 0.8,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
            Expanded(
              child: SliderTheme(
                data: SliderTheme.of(context).copyWith(
                  trackHeight: 2,
                  thumbShape: const RoundSliderThumbShape(
                    enabledThumbRadius: 5,
                  ),
                ),
                child: Slider(
                  value: value.clamp(min, max),
                  min: min,
                  max: max,
                  onChanged: onChanged,
                ),
              ),
            ),
            SizedBox(
              width: 42,
              child: Text(
                "${value.toStringAsFixed(displayDecimals)}$unit",
                textAlign: TextAlign.right,
                style: const TextStyle(
                  fontSize: 9,
                  color: Colors.white,
                  fontFamily: 'monospace',
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildJoystickColumn() {
    return GlassContainer(
      width: 250,
      padding: EdgeInsets.zero,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(16),
        child: SingleChildScrollView(
          controller: _joystickScrollController,
          physics: _joystickTouchCount > 0
              ? const NeverScrollableScrollPhysics()
              : const ClampingScrollPhysics(),
          padding: const EdgeInsets.fromLTRB(16, 16, 16, 32),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.start,
            children: [
              const Center(
                child: Text(
                  "STEERING",
                  style: TextStyle(
                    fontSize: 12,
                    letterSpacing: 1.5,
                    color: Colors.white54,
                    fontWeight: FontWeight.w300,
                  ),
                ),
              ),
              const SizedBox(height: 28),

              // YAW section header
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text(
                    "YAW",
                    style: TextStyle(
                      color: _yawLocked ? Colors.white38 : Colors.white,
                      fontSize: 11,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  GestureDetector(
                    behavior: HitTestBehavior.opaque,
                    onTap: () {
                      setState(() => _yawLocked = !_yawLocked);
                    },
                    child: Padding(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 8,
                        vertical: 6,
                      ),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Text(
                            _yawLocked ? "LOCKED" : "UNLOCKED",
                            style: TextStyle(
                              fontSize: 9,
                              letterSpacing: 1.0,
                              color: _yawLocked
                                  ? Colors.redAccent
                                  : Colors.white54,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          const SizedBox(width: 8),
                          Icon(
                            _yawLocked ? Icons.lock : Icons.lock_open_rounded,
                            size: 14,
                            color: _yawLocked
                                ? Colors.redAccent
                                : Colors.white54,
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 18),
              Listener(
                onPointerDown: (_) => setState(() => _joystickTouchCount++),
                onPointerUp: (_) => setState(
                  () => _joystickTouchCount = (_joystickTouchCount - 1).clamp(
                    0,
                    99,
                  ),
                ),
                onPointerCancel: (_) => setState(
                  () => _joystickTouchCount = (_joystickTouchCount - 1).clamp(
                    0,
                    99,
                  ),
                ),
                child: SizedBox(
                  height: 175,
                  width: 175,
                  child: IgnorePointer(
                    ignoring: _yawLocked,
                    child: AnimatedOpacity(
                      duration: const Duration(milliseconds: 200),
                      opacity: _yawLocked ? 0.3 : 1.0,
                      child: _YawDial(
                        angleNotifier: _yawAngleNotifier,
                        locked: _yawLocked,
                        onChanged: (v) {
                          _latestR = v;
                          if ((v - _jr).abs() > 0.03) {
                            setState(() => _jr = v);
                          }
                          if (_connected) _startSending();
                        },
                      ),
                    ),
                  ),
                ),
              ),

              const SizedBox(height: 28),

              // divider
              Container(height: 1, color: Colors.white.withOpacity(0.08)),

              const SizedBox(height: 20),

              _buildSequencePanel(),

              const SizedBox(height: 20),

              Container(height: 1, color: Colors.white.withOpacity(0.08)),

              const SizedBox(height: 20),

              // TILT section header
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text(
                    "TILT",
                    style: TextStyle(
                      color: _tiltLocked ? Colors.white38 : Colors.white,
                      fontSize: 11,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  GestureDetector(
                    behavior: HitTestBehavior.opaque,
                    onTap: () {
                      final newLocked = !_tiltLocked;

                      if (newLocked && _sequencePlaying) {
                        _stopSequence(resetStick: false);
                      }

                      setState(() => _tiltLocked = newLocked);

                      if (newLocked) {
                        _tiltJoyKey.currentState?.latchAtCurrent();
                      }
                    },
                    child: Padding(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 8,
                        vertical: 6,
                      ),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Text(
                            _tiltLocked ? "LOCKED" : "UNLOCKED",
                            style: TextStyle(
                              fontSize: 9,
                              letterSpacing: 1.0,
                              color: _tiltLocked
                                  ? Colors.redAccent
                                  : Colors.white54,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          const SizedBox(width: 8),
                          Icon(
                            _tiltLocked ? Icons.lock : Icons.lock_open_rounded,
                            size: 14,
                            color: _tiltLocked
                                ? Colors.redAccent
                                : Colors.white54,
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 8),

              IgnorePointer(
                ignoring: _tiltLocked || _sequencePlaying,
                child: AnimatedOpacity(
                  duration: const Duration(milliseconds: 200),
                  opacity: (_tiltLocked || _sequencePlaying) ? 0.3 : 1.0,
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      _buildModeChip(
                        "JOYSTICK",
                        !_tiltModeAccel,
                        () => _setTiltMode(false),
                      ),
                      const SizedBox(width: 8),
                      _buildModeChip(
                        "ACCEL",
                        _tiltModeAccel,
                        () => _setTiltMode(true),
                      ),
                    ],
                  ),
                ),
              ),
              const SizedBox(height: 18),
              Listener(
                onPointerDown: (_) => setState(() => _joystickTouchCount++),
                onPointerUp: (_) => setState(
                  () => _joystickTouchCount = (_joystickTouchCount - 1).clamp(
                    0,
                    99,
                  ),
                ),
                onPointerCancel: (_) => setState(
                  () => _joystickTouchCount = (_joystickTouchCount - 1).clamp(
                    0,
                    99,
                  ),
                ),
                child: SizedBox(
                  height: 175,
                  width: 175,
                  child: IgnorePointer(
                    ignoring: _tiltLocked || _tiltModeAccel || _sequencePlaying,
                    child: AnimatedOpacity(
                      duration: const Duration(milliseconds: 200),
                      opacity: _tiltLocked ? 0.3 : 1.0,
                      child: _FixedBaseJoystick(
                        key: _tiltJoyKey,
                        externalKnob: (_sequencePlaying || _tiltModeAccel)
                            ? Offset(_jx, -_jy)
                            : null,
                        locked: _tiltLocked,
                        onStart: () {
                          _joystickActive = true;
                          _startSending();
                        },
                        onChanged: (x, y) {
                          _latestX = x;
                          _latestY = y;

                          final dx = (x - _jx).abs();
                          final dy = (y - _jy).abs();
                          if (dx > 0.03 || dy > 0.03) {
                            setState(() {
                              _jx = x;
                              _jy = y;
                            });
                          }
                        },
                        onRelease: () {
                          _joystickActive = false;
                        },
                      ),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildCenterPanel() {
    return Column(
      children: [
        Expanded(
          child: GlassContainer(
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
            child: Column(
              children: [
                const SizedBox(height: 4),
                const Center(
                  child: Text(
                    "PID CONFIGURATION",
                    style: TextStyle(
                      fontSize: 12,
                      letterSpacing: 1.5,
                      color: Colors.white54,
                    ),
                  ),
                ),
                const SizedBox(height: 12),

                Expanded(
                  child: ListView(
                    padding: EdgeInsets.zero,
                    children: [
                      _buildSectionHeader("PITCH & ROLL", _pidPitchLocked, () {
                        setState(() => _pidPitchLocked = !_pidPitchLocked);
                      }),
                      _buildPidRow(
                        "P",
                        _pidPitchLocked,
                        (_pActual / _pMult).clamp(0.0, 1.0),
                        _defP,
                        _pMult,
                        _defaultPMult,
                        (v) => setState(() => _pActual = v * _pMult),
                        (m) => setState(() => _pMult = m),
                      ),
                      _buildPidRow(
                        "I",
                        _pidPitchLocked,
                        (_iActual / _iMult).clamp(0.0, 1.0),
                        _defI,
                        _iMult,
                        _defaultIMult,
                        (v) => setState(() => _iActual = v * _iMult),
                        (m) => setState(() => _iMult = m),
                      ),
                      _buildPidRow(
                        "D",
                        _pidPitchLocked,
                        (_dActual / _dMult).clamp(0.0, 1.0),
                        _defD,
                        _dMult,
                        _defaultDMult,
                        (v) => setState(() => _dActual = v * _dMult),
                        (m) => setState(() => _dMult = m),
                      ),

                      _buildSectionHeader("OUTER LOOP", _pidOuterLocked, () {
                        setState(() => _pidOuterLocked = !_pidOuterLocked);
                      }),
                      _buildPidRow(
                        "P",
                        _pidOuterLocked,
                        (_angleKPActual / _angleKPMult).clamp(0.0, 1.0),
                        _defAngleKP,
                        _angleKPMult,
                        _defaultAngleKPMult,
                        (v) =>
                            setState(() => _angleKPActual = v * _angleKPMult),
                        (m) => setState(() => _angleKPMult = m),
                      ),

                      _buildSectionHeader("YAW", _pidYawLocked, () {
                        setState(() => _pidYawLocked = !_pidYawLocked);
                      }),
                      const SizedBox(height: 8),
                      _buildPidRow(
                        "P",
                        _pidYawLocked,
                        (_p2Actual / _p2Mult).clamp(0.0, 1.0),
                        _defP2,
                        _p2Mult,
                        _defaultP2Mult,
                        (v) => setState(() => _p2Actual = v * _p2Mult),
                        (m) => setState(() => _p2Mult = m),
                      ),
                      _buildPidRow(
                        "I",
                        _pidYawLocked,
                        (_i2Actual / _i2Mult).clamp(0.0, 1.0),
                        _defI2,
                        _i2Mult,
                        _defaultI2Mult,
                        (v) => setState(() => _i2Actual = v * _i2Mult),
                        (m) => setState(() => _i2Mult = m),
                      ),
                      _buildPidRow(
                        "D",
                        _pidYawLocked,
                        (_d2Actual / _d2Mult).clamp(0.0, 1.0),
                        _defD2,
                        _d2Mult,
                        _defaultD2Mult,
                        (v) => setState(() => _d2Actual = v * _d2Mult),
                        (m) => setState(() => _d2Mult = m),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildSectionHeader(String title, bool isLocked, VoidCallback onLock) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 8.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            title,
            style: TextStyle(
              color: isLocked ? Colors.white38 : Colors.white,
              fontSize: 11,
              fontWeight: FontWeight.bold,
            ),
          ),
          GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTap: onLock,
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    isLocked ? "LOCKED" : "UNLOCKED",
                    style: TextStyle(
                      fontSize: 9,
                      letterSpacing: 1.0,
                      color: isLocked ? Colors.redAccent : Colors.white54,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(width: 8),
                  Icon(
                    isLocked ? Icons.lock : Icons.lock_open_rounded,
                    size: 14,
                    color: isLocked ? Colors.redAccent : Colors.white54,
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildPidRow(
    String label,
    bool isLocked,
    double value,
    double defaultValue,
    double multiplier,
    double defaultMultiplier,
    ValueChanged<double> onChanged,
    ValueChanged<double> onMultiplierChanged,
  ) {
    final multipliers = [0.01, 0.1, 1.0, 10.0, 100.0];
    final labels = ["/100", "/10", "x1", "x10", "x100"];

    return IgnorePointer(
      ignoring: isLocked,
      child: AnimatedOpacity(
        duration: const Duration(milliseconds: 200),
        opacity: isLocked ? 0.3 : 1.0,
        child: Padding(
          padding: const EdgeInsets.only(bottom: 12.0),
          child: Column(
            children: [
              SizedBox(
                height: 36,
                child: Row(
                  children: [
                    SizedBox(
                      width: 20,
                      child: Text(
                        label,
                        style: const TextStyle(
                          fontWeight: FontWeight.bold,
                          color: Colors.white70,
                        ),
                      ),
                    ),
                    Expanded(
                      child: Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 24.0),
                        child: LayoutBuilder(
                          builder: (context, constraints) {
                            final width = constraints.maxWidth;
                            final usableWidth = width - 48;

                            final defaultActual = defaultValue;
                            final recommended = (defaultActual / multiplier)
                                .clamp(0.0, 1.0);

                            final offset = 24 + (usableWidth * recommended);

                            return Stack(
                              alignment: Alignment.centerLeft,
                              children: [
                                Positioned(
                                  left: offset - 2,
                                  child: Container(
                                    width: 4,
                                    height: 18,
                                    decoration: BoxDecoration(
                                      color: Colors.amber.withOpacity(0.75),
                                      borderRadius: BorderRadius.circular(2),
                                      boxShadow: [
                                        BoxShadow(
                                          color: Colors.amber.withOpacity(0.35),
                                          blurRadius: 6,
                                          spreadRadius: 1,
                                        ),
                                      ],
                                    ),
                                  ),
                                ),
                                Slider(
                                  value: value,
                                  min: 0.0,
                                  max: 1.0,
                                  divisions: 1000,
                                  onChanged: (v) => onChanged(v),
                                ),
                              ],
                            );
                          },
                        ),
                      ),
                    ),
                    SizedBox(
                      width: 50,
                      child: Text(
                        (value * multiplier).toStringAsFixed(
                          multiplier < 1 ? 4 : 2,
                        ),
                        textAlign: TextAlign.right,
                        style: const TextStyle(
                          fontFamily: 'monospace',
                          fontSize: 10,
                          color: Colors.white,
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              SingleChildScrollView(
                scrollDirection: Axis.horizontal,
                child: Wrap(
                  spacing: 8,
                  children: List.generate(multipliers.length, (index) {
                    final m = multipliers[index];
                    final isSelected = (m == multiplier);
                    final isDefault = (m == defaultMultiplier);
                    const yellowWidth = 1.5;
                    return GestureDetector(
                      onTap: () => onMultiplierChanged(m),
                      child: Container(
                        padding: EdgeInsets.symmetric(
                          horizontal: 6,
                          vertical: isDefault ? 3 - yellowWidth : 3,
                        ),
                        decoration: BoxDecoration(
                          color: isSelected ? Colors.white : Colors.white10,
                          borderRadius: BorderRadius.circular(4),
                          border: isDefault
                              ? Border.all(
                                  color: Colors.amber.withOpacity(0.6),
                                  width: yellowWidth,
                                )
                              : null,
                        ),
                        child: Text(
                          labels[index],
                          style: TextStyle(
                            fontSize: 9,
                            fontWeight: FontWeight.bold,
                            color: isSelected ? Colors.black : Colors.white54,
                          ),
                        ),
                      ),
                    );
                  }),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _YawDial extends StatelessWidget {
  final ValueNotifier<double> angleNotifier;
  final ValueChanged<double> onChanged;
  final bool locked;

  const _YawDial({
    super.key,
    required this.angleNotifier,
    required this.onChanged,
    required this.locked,
  });

  void _handlePan(Offset local, double size) {
    if (locked) return;
    final center = Offset(size / 2, size / 2);
    final vec = local - center;
    if (vec.distanceSquared < 100) return;

    double val = (vec.direction + pi / 2) / pi;
    if (val > 1.0) val -= 2.0;

    final clamped = val.clamp(-1.0, 1.0);
    angleNotifier.value = clamped; // update notifier directly — no setState
    onChanged(clamped);
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, c) {
        final size = min(c.maxWidth, c.maxHeight);
        return GestureDetector(
          onPanStart: locked ? null : (d) => _handlePan(d.localPosition, size),
          onPanUpdate: locked ? null : (d) => _handlePan(d.localPosition, size),
          // AnimatedBuilder listens to the notifier and repaints only this widget
          child: AnimatedBuilder(
            animation: angleNotifier,
            builder: (_, __) => CustomPaint(
              size: Size(size, size),
              painter: _YawDialPainter(angle: angleNotifier.value * pi),
            ),
          ),
        );
      },
    );
  }
}

class _YawDialPainter extends CustomPainter {
  final double angle;
  _YawDialPainter({required this.angle});

  @override
  void paint(Canvas canvas, Size size) {
    final c = Offset(size.width / 2, size.height / 2);
    final r = size.width * 0.45;

    final paintBg = Paint()
      ..color = Colors.white.withOpacity(0.05)
      ..style = PaintingStyle.fill;
    final paintStroke = Paint()
      ..color = Colors.white.withOpacity(0.2)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    canvas.drawCircle(c, r, paintBg);
    canvas.drawCircle(c, r, paintStroke);

    final tickPaint = Paint()
      ..color = Colors.white.withOpacity(0.1)
      ..strokeWidth = 1;

    for (int i = 0; i < 12; i++) {
      final a = i * (2 * pi / 12);
      canvas.drawLine(
        c + Offset.fromDirection(a, r * 0.7),
        c + Offset.fromDirection(a, r * 0.9),
        tickPaint,
      );
    }

    final labelStyle = TextStyle(
      color: Colors.white.withOpacity(0.35),
      fontSize: 9,
      fontWeight: FontWeight.w600,
    );

    const stepDeg = 45;
    for (int i = 0; i < 360; i += stepDeg) {
      var deg = i;
      final dir = (i * pi / 180) - (pi / 2);
      final pos = c + Offset.fromDirection(dir, r * 1.15);

      final tp = TextPainter(
        text: TextSpan(text: "$deg°", style: labelStyle),
        textDirection: TextDirection.ltr,
      )..layout();

      canvas.drawParagraph(
        (ParagraphBuilder(ParagraphStyle(textAlign: TextAlign.center))
              ..pushStyle(labelStyle.getTextStyle())
              ..addText("$deg°"))
            .build()
          ..layout(ParagraphConstraints(width: tp.width)),
        Offset(pos.dx - tp.width / 2, pos.dy - tp.height / 2),
      );
    }

    final visualAngle = angle - pi / 2;
    final arrowLen = r * 0.8;
    final tip = c + Offset.fromDirection(visualAngle, arrowLen);

    final arrowPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 3
      ..strokeCap = StrokeCap.round
      ..style = PaintingStyle.stroke;

    canvas.drawLine(c, tip, arrowPaint);

    final path = Path();
    final wingAngle = 30 * pi / 180;
    final headLen = 15.0;

    path.moveTo(tip.dx, tip.dy);
    path.lineTo(
      tip.dx - headLen * cos(visualAngle - wingAngle),
      tip.dy - headLen * sin(visualAngle - wingAngle),
    );
    path.moveTo(tip.dx, tip.dy);
    path.lineTo(
      tip.dx - headLen * cos(visualAngle + wingAngle),
      tip.dy - headLen * sin(visualAngle + wingAngle),
    );

    canvas.drawPath(path, arrowPaint);
    canvas.drawCircle(c, 4, Paint()..color = Colors.white);
  }

  @override
  bool shouldRepaint(covariant _YawDialPainter old) => old.angle != angle;
}

// throttle track shape
class _GlassTrackShape extends RoundedRectSliderTrackShape {
  final double activeValue;

  _GlassTrackShape({this.activeValue = 0.0});

  @override
  Rect getPreferredRect({
    required RenderBox parentBox,
    Offset offset = Offset.zero,
    required SliderThemeData sliderTheme,
    bool isEnabled = false,
    bool isDiscrete = false,
  }) {
    final double trackHeight = sliderTheme.trackHeight ?? 0;
    final double trackLeft = offset.dx;
    final double trackTop =
        offset.dy + (parentBox.size.height - trackHeight) / 2;
    final double trackWidth = parentBox.size.width;
    return Rect.fromLTWH(trackLeft, trackTop, trackWidth, trackHeight);
  }

  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required TextDirection textDirection,
    required Offset thumbCenter,
    Offset? secondaryOffset,
    bool isDiscrete = false,
    bool isEnabled = false,
    double additionalActiveTrackHeight = 0,
  }) {
    final Rect trackRect = getPreferredRect(
      parentBox: parentBox,
      offset: offset,
      sliderTheme: sliderTheme,
      isEnabled: isEnabled,
      isDiscrete: isDiscrete,
    );

    final Paint paintBg = Paint()
      ..color = Colors.white.withOpacity(0.05)
      ..style = PaintingStyle.fill;

    final Paint paintBorder = Paint()
      ..color = Colors.white.withOpacity(0.2)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;

    context.canvas.drawRRect(
      RRect.fromRectAndRadius(trackRect, const Radius.circular(8)),
      paintBg,
    );
    context.canvas.drawRRect(
      RRect.fromRectAndRadius(trackRect, const Radius.circular(8)),
      paintBorder,
    );

    final Paint paintFill = Paint()..color = Colors.white;
    final double fillWidth = trackRect.width * activeValue;

    final fillRect = Rect.fromLTRB(
      trackRect.left,
      trackRect.top,
      trackRect.left + fillWidth,
      trackRect.bottom,
    );

    context.canvas.drawRRect(
      RRect.fromRectAndRadius(fillRect, const Radius.circular(8)),
      paintFill,
    );

    final double liftX = trackRect.left + trackRect.width * 0.33;

    const markerW = 4.0;
    final markerH = trackRect.height * 1.2;
    final markerTop = trackRect.top + (trackRect.height - markerH) * 0.5;

    final markerRect = RRect.fromRectAndRadius(
      Rect.fromLTWH(liftX - markerW / 2, markerTop, markerW, markerH),
      const Radius.circular(2),
    );

    final glowPaint = Paint()
      ..color = Colors.amber.withOpacity(0.35)
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 6);

    final barPaint = Paint()
      ..color = Colors.amber.withOpacity(0.75)
      ..style = PaintingStyle.fill;

    context.canvas.drawRRect(markerRect, glowPaint);
    context.canvas.drawRRect(markerRect, barPaint);
  }
}

class _FixedBaseJoystick extends StatefulWidget {
  final void Function(double x, double y) onChanged;
  final VoidCallback? onStart;
  final VoidCallback? onRelease;
  final bool locked;

  final Offset? externalKnob;

  const _FixedBaseJoystick({
    super.key,
    required this.onChanged,
    this.onStart,
    this.onRelease,
    required this.locked,
    this.externalKnob,
  });

  @override
  State<_FixedBaseJoystick> createState() => _FixedBaseJoystickState();
}

class _FixedBaseJoystickState extends State<_FixedBaseJoystick> {
  Offset _knob = Offset.zero;
  double _radius = 0;
  bool _latched = false;
  bool _gestureLatched = false;

  void latchAtCurrent() {
    if (_radius <= 0) return;
    setState(() {
      _latched = true;
      _gestureLatched = true;
    });

    final nx = (_knob.dx / _radius).clamp(-1.0, 1.0);
    final ny = (-_knob.dy / _radius).clamp(-1.0, 1.0);
    widget.onChanged(nx.toDouble(), ny.toDouble());
  }

  void syncFromExternal(double normX, double normY) {
    if (_radius <= 0) return;
    setState(() {
      _knob = Offset(normX * _radius, -normY * _radius);
      _latched = true;
      _gestureLatched = false;
    });
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, c) {
        final size = min(c.maxWidth, c.maxHeight);
        _radius = size * 0.45;
        final knobRadius = size * 0.15;

        final displayKnob = widget.externalKnob != null
            ? Offset(
                widget.externalKnob!.dx * _radius,
                -widget.externalKnob!.dy * _radius,
              )
            : _knob;

        return GestureDetector(
          onPanStart: widget.locked
              ? null
              : (d) {
                  widget.onStart?.call();
                  _gestureLatched = false;
                  _latched = false;
                  _updateFromLocal(d.localPosition, size);
                },
          onPanUpdate: widget.locked
              ? null
              : (d) => _updateFromLocal(d.localPosition, size),
          onPanEnd: (_) {
            _gestureLatched = false;

            if (widget.locked || _latched) {
              widget.onRelease?.call();
              return;
            }

            setState(() => _knob = Offset.zero);
            widget.onChanged(0, 0);
            widget.onRelease?.call();
          },
          child: CustomPaint(
            painter: _JoystickPainter(
              baseRadius: _radius,
              knobCenter: displayKnob,
              knobRadius: knobRadius,
            ),
            child: const SizedBox.expand(),
          ),
        );
      },
    );
  }

  void _updateFromLocal(Offset p, double size) {
    if (widget.locked) return;
    if (_gestureLatched) return;

    final center = Offset(size / 2, size / 2);
    final v = p - center;
    final dist = v.distance;

    if (dist > _radius) {
      final snappedAngle = _nearestOctant(v.direction);
      final snappedVec = Offset.fromDirection(snappedAngle, _radius);
      setState(() {
        _knob = snappedVec;
        _latched = true;
        _gestureLatched = true;
      });
      final nx = (_knob.dx / _radius).clamp(-1.0, 1.0);
      final ny = (-_knob.dy / _radius).clamp(-1.0, 1.0);
      widget.onChanged(nx.toDouble(), ny.toDouble());
      return;
    }

    if (_latched) return;

    setState(() => _knob = v);
    final nx = (v.dx / _radius).clamp(-1.0, 1.0);
    final ny = (-v.dy / _radius).clamp(-1.0, 1.0);
    widget.onChanged(nx.toDouble(), ny.toDouble());
  }

  double _nearestOctant(double angle) {
    double a = angle;
    while (a <= -pi) a += 2 * pi;
    while (a > pi) a -= 2 * pi;
    const step = pi / 4;
    final idx = (a / step).round();
    return idx * step.toDouble();
  }
}

class _JoystickPainter extends CustomPainter {
  final double baseRadius;
  final double knobRadius;
  final Offset knobCenter;
  _JoystickPainter({
    required this.baseRadius,
    required this.knobCenter,
    required this.knobRadius,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);

    final ringStroke = Colors.white.withOpacity(0.3);
    final knobColor = Colors.white.withOpacity(0.2);
    final knobStroke = Colors.white;

    canvas.drawCircle(
      center,
      baseRadius,
      Paint()..color = Colors.white.withOpacity(0.02),
    );

    final ringPaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..color = ringStroke;

    final tickPaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1
      ..color = Colors.white.withOpacity(0.2);

    canvas.drawCircle(center, baseRadius, ringPaint);

    canvas.drawLine(
      center - Offset(baseRadius / 2, 0),
      center + Offset(baseRadius / 2, 0),
      tickPaint,
    );
    canvas.drawLine(
      center - Offset(0, baseRadius / 2),
      center + Offset(0, baseRadius / 2),
      tickPaint,
    );

    for (int i = 0; i < 8; i++) {
      final a = i * pi / 4;
      final p1 = center + Offset.fromDirection(a, baseRadius * 0.9);
      final p2 = center + Offset.fromDirection(a, baseRadius);
      canvas.drawLine(p1, p2, tickPaint);
    }

    final labelStyle = TextStyle(
      color: Colors.white.withOpacity(0.45),
      fontSize: 10,
      fontWeight: FontWeight.w700,
      letterSpacing: 1.2,
    );

    void drawLabel(String text, Offset pos, {double rotateRad = 0.0}) {
      final tp = TextPainter(
        text: TextSpan(text: text, style: labelStyle),
        textDirection: TextDirection.ltr,
      )..layout();

      final shadow = TextPainter(
        text: TextSpan(
          text: text,
          style: labelStyle.copyWith(color: Colors.white.withOpacity(0.12)),
        ),
        textDirection: TextDirection.ltr,
      )..layout();

      canvas.save();
      canvas.translate(pos.dx, pos.dy);
      if (rotateRad != 0.0) canvas.rotate(rotateRad);

      shadow.paint(canvas, Offset(-tp.width / 2 + 0.6, -tp.height / 2 + 0.6));
      tp.paint(canvas, Offset(-tp.width / 2, -tp.height / 2));

      canvas.restore();
    }

    final pad = baseRadius * 0.14;
    drawLabel("FWD", center + Offset(0, -(baseRadius + pad)));
    drawLabel("BACK", center + Offset(0, (baseRadius + pad)));

    final rot90 = pi / 2;
    drawLabel(
      "LEFT",
      center + Offset(-(baseRadius + pad), 0),
      rotateRad: -rot90,
    );
    drawLabel(
      "RIGHT",
      center + Offset((baseRadius + pad), 0),
      rotateRad: rot90,
    );

    final knobPos = center + knobCenter;

    final shadowPaint = Paint()
      ..color = Colors.white.withOpacity(0.1)
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 10);
    canvas.drawCircle(knobPos, knobRadius, shadowPaint);

    final kPaint = Paint()..color = knobColor;
    canvas.drawCircle(knobPos, knobRadius, kPaint);

    final kStroke = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..color = knobStroke;
    canvas.drawCircle(knobPos, knobRadius, kStroke);

    canvas.drawCircle(knobPos, 3, Paint()..color = Colors.white);
  }

  @override
  bool shouldRepaint(covariant _JoystickPainter oldDelegate) {
    return oldDelegate.knobCenter != knobCenter ||
        oldDelegate.baseRadius != baseRadius ||
        oldDelegate.knobRadius != knobRadius;
  }
}
