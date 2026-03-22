# Copyright (c) 2025-2026 Hannes Göök
# MIT License - PidraQRL
# https://github.com/hannesgook/pidraqrl

import asyncio
import socket
import struct
import threading
import time
import logging
import numpy as np
from bleak import BleakClient, BleakScanner
from rl.sac_agent import SACAgent

logging.basicConfig(level=logging.INFO, format="%(asctime)s [BRIDGE] %(message)s")
log = logging.getLogger(__name__)

# BLE & UDP
DEVICE_NAME = "ESP32-FC"
SERVICE_UUID = "fc000000-0000-0000-0000-000000000001"
TX_CHAR_UUID = "fc000000-0000-0000-0000-000000000002"
RX_CHAR_UUID = "fc000000-0000-0000-0000-000000000003"

IMU_PKT_FMT = "<BfffffffI"
IMU_PKT_SIZE = struct.calcsize(IMU_PKT_FMT) # 33 bytes

UDP_IMU_FMT = "<dfffffff" # timestamp(double) + 7 floats = 36 bytes
UNITY_IP = "127.0.0.1"
UNITY_PORT = 5005
CMD_PORT = 5006
STATUS_PORT = 5007

# ranges
ANGLE_KP_MIN, ANGLE_KP_MAX = 3.3, 3.9
RATE_KP_MIN,  RATE_KP_MAX = 0.45, 0.65
SAFE_ANGLE_KP = 3.5
SAFE_ROLL_P = 0.52
SAFE_ROLL_I = 0.0
SAFE_ROLL_D = 0.0611
SAFE_STICK_ROLL = 0.0
SAFE_THROTTLE_US = 1150
THROTTLE_US = 1150

# rl
EPISODE_DURATION = 8.0
SETTLE_DURATION = 1.5
UPDATES_PER_STEP = 2
WARMUP_EPISODES = 15
SAVE_EVERY = 20
MAX_EPISODES = 10_000
IMU_LOOP_DT = 1.0 / 50
MAX_ANGLE_DEG = 45.0
MAX_RATE_DPS = 300.0
TARGET_RANGE_DEG = 30.0
W_TRACKING = 2.0
W_RATE = 0.3
W_GAIN_CHANGE = 0.0
STABILITY_BONUS = 3.0
STABLE_DEG = 3.0
CRASH_PENALTY = -10.0

HEARTBEAT_HZ = 10
HEARTBEAT_INTERVAL = 1.0 / HEARTBEAT_HZ

# BLE refresh
# Resend 'C' 0x01 at this interval during training so the ESP32's claim
# timeout (2 s) never fires while actively training.
CLAIM_REFRESH_INTERVAL = 0.5 # seconds

# build packet
def _map8(value: float, in_min: float, in_max: float) -> int:
    v = (value - in_min) / (in_max - in_min) * 255.0
    return int(max(0, min(255, round(v))))

def build_s_packet(throttle_us: int, stick_roll: float, stick_pitch: float, stick_yaw: float, roll_p: float, roll_i: float, roll_d: float, yaw_p:  float, yaw_i:  float, yaw_d:  float, angle_kp: float) -> bytes:
    payload = bytearray(34)
    payload[0] = ord('S')
    payload[1] = _map8(throttle_us, 1000.0, 2000.0)
    payload[2] = _map8(stick_roll, -1.0, 1.0)
    payload[3] = _map8(stick_pitch, -1.0, 1.0)
    payload[4] = _map8(stick_yaw, -1.0, 1.0)
    struct.pack_into('<f', payload,  5, roll_p)
    struct.pack_into('<f', payload,  9, roll_i)
    struct.pack_into('<f', payload, 13, roll_d)
    struct.pack_into('<f', payload, 17, yaw_p)
    struct.pack_into('<f', payload, 21, yaw_i)
    struct.pack_into('<f', payload, 25, yaw_d)
    struct.pack_into('<f', payload, 29, angle_kp)
    chk = 0
    for b in payload[1:33]:
        chk ^= b
    payload[33] = chk
    return bytes(payload)

def build_claim_packet(claim: bool) -> bytes:
    # build a 'C' packet: C + 0x01 (claim) or C + 0x00 (release).
    return bytes([ord('C'), 0x01 if claim else 0x00])

def _safe_pkt() -> bytes:
    return build_s_packet(
        throttle_us=SAFE_THROTTLE_US,
        stick_roll=0.0, stick_pitch=0.0, stick_yaw=0.0,
        roll_p=SAFE_ROLL_P, roll_i=SAFE_ROLL_I, roll_d=SAFE_ROLL_D,
        yaw_p=0.0, yaw_i=0.0, yaw_d=0.0,
        angle_kp=SAFE_ANGLE_KP,
    )

def selftest_packet():
    pkt = build_s_packet(
        throttle_us=1150,
        stick_roll=0.5, stick_pitch=0.0, stick_yaw=0.0,
        roll_p=1.2, roll_i=0.01, roll_d=0.005,
        yaw_p=0.8,  yaw_i=0.0,   yaw_d=0.0,
        angle_kp=10.0,
    )
    assert len(pkt) == 34, f"bad length {len(pkt)}"
    assert pkt[0] == ord('S'), f"bad marker 0x{pkt[0]:02X} (want 0x53)"

    chk = 0
    for b in pkt[1:33]:
        chk ^= b
    assert chk == pkt[33], f"bad checksum 0x{chk:02X} != 0x{pkt[33]:02X}"

    roll_p_rt, roll_i_rt, roll_d_rt = struct.unpack_from('<fff', pkt,  5)
    yaw_p_rt,  yaw_i_rt,  yaw_d_rt = struct.unpack_from('<fff', pkt, 17)
    angle_kp_rt, = struct.unpack_from('<f',   pkt, 29)
    assert abs(roll_p_rt - 1.2) < 1e-5, f"roll_p  fail: {roll_p_rt}"
    assert abs(roll_i_rt - 0.01) < 1e-5, f"roll_i  fail: {roll_i_rt}"
    assert abs(roll_d_rt - 0.005) < 1e-5, f"roll_d  fail: {roll_d_rt}"
    assert abs(angle_kp_rt - 10.0) < 1e-5, f"angleKP fail: {angle_kp_rt}"

    stick_roll_rt = pkt[2] / 255.0 * 2.0 - 1.0
    assert abs(stick_roll_rt - 0.5) < 2/255, f"stick_roll fail: {stick_roll_rt}"

    # test claim packets
    c_claim = build_claim_packet(True)
    c_release = build_claim_packet(False)
    assert c_claim == b'C\x01', f"claim packet fail: {c_claim.hex()}"
    assert c_release == b'C\x00', f"release packet fail: {c_release.hex()}"

    log.info("[SELF-TEST] S-packet OK: %s", pkt.hex())
    log.info("[SELF-TEST] C-packets OK")

class DroneInterface:
    def __init__(self, unity_ip=UNITY_IP, unity_port=UNITY_PORT):
        self._lock = threading.Lock()
        self._roll = 0.0
        self._rate = 0.0
        self._target_roll = 0.0
        self._connected = False

        self.current_throttle_us = 1000
        self.current_stick_roll = SAFE_STICK_ROLL
        self.current_stick_pitch = 0.0
        self.current_stick_yaw = 0.0
        self.current_angle_kp = SAFE_ANGLE_KP
        self.current_rate_kp = SAFE_ROLL_P
        self.current_roll_p = SAFE_ROLL_P
        self.current_roll_i = SAFE_ROLL_I
        self.current_roll_d = SAFE_ROLL_D
        self.current_yaw_p = 0.0
        self.current_yaw_i = 0.0
        self.current_yaw_d = 0.0

        self._training_active = False
        self._training_lock = threading.Lock()
        self._last_claim_time = 0.0

        # Pre-built S-packet swapped atomically, heartbeat sends this
        self._pending_lock = threading.Lock()
        self._pending_pkt = _safe_pkt()

        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._unity_addr = (unity_ip, unity_port)

        self._ble_loop = asyncio.new_event_loop()
        self._client = None
        self._last_packet_time = time.monotonic()
        threading.Thread(target=self._ble_thread, daemon=True).start()

        self._hb_loop = asyncio.new_event_loop()
        threading.Thread(target=self._hb_thread, daemon=True).start()

    def is_connected(self) -> bool:
        return self._connected

    def read_imu(self) -> dict:
        with self._lock:
            return {
                "roll_angle": self._roll,
                "roll_rate": self._rate,
                "target_roll_angle": self._target_roll,
            }

    # claim / release
    def claim_control(self):
        with self._training_lock:
            self._training_active = True
        pkt = build_claim_packet(True)
        self._queue_raw(pkt)
        self._last_claim_time = time.monotonic()
        log.info("[DRONE] Sent BLE CLAIM")

    def release_control(self):
        with self._training_lock:
            self._training_active = False
        pkt = build_claim_packet(False)
        self._queue_raw(pkt)
        log.info("[DRONE] Sent BLE RELEASE")

    def _queue_raw(self, pkt: bytes):
        if not (self._client and self._client.is_connected):
            return
        fut = asyncio.run_coroutine_threadsafe(self._send(pkt), self._ble_loop)
        try:
            fut.result(timeout=0.1)
        except Exception as e:
            log.warning("[DRONE] Raw send failed: %s", e)

    def send_command(self, stick_roll: float, angle_kp: float, roll_p: float):
        pkt = build_s_packet(throttle_us=THROTTLE_US, stick_roll=stick_roll, stick_pitch=0.0, stick_yaw=0.0, roll_p=roll_p, roll_i=SAFE_ROLL_I, roll_d=SAFE_ROLL_D, yaw_p=0.0, yaw_i=0.0, yaw_d=0.0, angle_kp=angle_kp)
        self._commit(pkt, THROTTLE_US, stick_roll, angle_kp, roll_p, SAFE_ROLL_I, SAFE_ROLL_D)

    def send_safe_defaults(self):
        pkt = _safe_pkt()
        self._commit(pkt, SAFE_THROTTLE_US, SAFE_STICK_ROLL, SAFE_ANGLE_KP, SAFE_ROLL_P, SAFE_ROLL_I, SAFE_ROLL_D)

    def _commit(self, pkt, throttle_us, stick_roll, angle_kp, roll_p, roll_i, roll_d):
        with self._pending_lock:
            self._pending_pkt = pkt
        self.current_throttle_us = throttle_us
        self.current_stick_roll = stick_roll
        self.current_angle_kp = angle_kp
        self.current_rate_kp = roll_p
        self.current_roll_p = roll_p
        self.current_roll_i = roll_i
        self.current_roll_d = roll_d

    def normalise_gains(self) -> tuple:
        akn = 2.0 * (self.current_angle_kp - ANGLE_KP_MIN) / (ANGLE_KP_MAX - ANGLE_KP_MIN) - 1.0
        rkn = 2.0 * (self.current_roll_p   - RATE_KP_MIN)  / (RATE_KP_MAX  - RATE_KP_MIN)  - 1.0
        return float(akn), float(rkn)

    def scale_action(self, action: np.ndarray) -> tuple:
        angle_kp = ANGLE_KP_MIN + (action[0] + 1.0) * 0.5 * (ANGLE_KP_MAX - ANGLE_KP_MIN)
        roll_p = RATE_KP_MIN  + (action[1] + 1.0) * 0.5 * (RATE_KP_MAX  - RATE_KP_MIN)
        return float(angle_kp), float(roll_p)

    # heartbeat
    def _hb_thread(self):
        asyncio.set_event_loop(self._hb_loop)
        self._hb_loop.run_until_complete(self._hb_coro())

    async def _hb_coro(self):
        log.info("[HB] Heartbeat started at %d Hz", HEARTBEAT_HZ)
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL)
            if not (self._client and self._client.is_connected):
                continue

            # refresh
            with self._training_lock:
                training = self._training_active
            if training:
                now = time.monotonic()
                if (now - self._last_claim_time) >= CLAIM_REFRESH_INTERVAL:
                    claim_pkt = build_claim_packet(True)
                    fut = asyncio.run_coroutine_threadsafe(
                        self._send(claim_pkt), self._ble_loop)
                    try:
                        fut.result(timeout=0.08)
                        self._last_claim_time = now
                    except Exception:
                        pass

            # send S packet
            with self._pending_lock:
                pkt = self._pending_pkt
            fut = asyncio.run_coroutine_threadsafe(self._send(pkt), self._ble_loop)
            try:
                fut.result(timeout=0.08)
            except Exception as e:
                log.warning("[HB] Send failed: %s", e)

    def _ble_thread(self):
        asyncio.set_event_loop(self._ble_loop)
        self._ble_loop.run_until_complete(self._ble_loop_coro())

    async def _ble_loop_coro(self):
        while True:
            log.info("[BLE] Scanning for ESP32...")
            found_addr = None
            try:
                stop_event = asyncio.Event()

                def detection_callback(device, advertisement_data):
                    nonlocal found_addr
                    if found_addr:
                        return
                    name = advertisement_data.local_name or device.name or ""
                    uuids = [str(u).lower()
                             for u in (advertisement_data.service_uuids or [])]
                    service_match = SERVICE_UUID.lower() in uuids
                    if name or service_match:
                        log.info("[BLE] Seen: name='%s' addr=%s service_match=%s",
                                 name, device.address, service_match)
                    if DEVICE_NAME in name or service_match:
                        log.info("[BLE] Found ESP32 -> %s (name='%s')",
                                 device.address, name)
                        found_addr = device.address
                        stop_event.set()

                scanner = BleakScanner(detection_callback)
                await scanner.start()
                try:
                    await asyncio.wait_for(stop_event.wait(), timeout=10.0)
                except asyncio.TimeoutError:
                    pass
                finally:
                    await scanner.stop()

                if not found_addr:
                    log.warning("[BLE] Not found, retrying...")
                    await asyncio.sleep(2.0)
                    continue

                await asyncio.sleep(1.0)

                def on_disconnect(c):
                    log.warning("[BLE] Device disconnected (callback)")
                    self._connected = False
                    # auto-clear training flag so heartbeat stops refreshing claim
                    with self._training_lock:
                        self._training_active = False

                async with BleakClient(found_addr, timeout=15.0,
                                       disconnected_callback=on_disconnect) as client:
                    self._client = client
                    self._connected = True
                    self._last_packet_time = time.monotonic()
                    log.info("[BLE] Connected to %s", found_addr)

                    await client.start_notify(TX_CHAR_UUID, self._on_notify)

                    while client.is_connected and self._connected:
                        await asyncio.sleep(0.1)
                        if time.monotonic() - self._last_packet_time > 3.0:
                            log.warning("[BLE] No packets for 3 s, silent disconnect")
                            self._connected = False
                            break

                    log.warning("[BLE] Connection lost, leaning up")

            except asyncio.CancelledError:
                log.warning("[BLE] Cancelled, retrying in 3 s...")
                await asyncio.sleep(3.0)
                continue
            except Exception as e:
                log.error("[BLE] Error: %s", e)
            finally:
                self._connected = False
                self._client = None

            log.info("[BLE] Disconnected, reconnecting in 2 s...")
            await asyncio.sleep(2.0)

    def _on_notify(self, _sender, data: bytearray):
        if len(data) < IMU_PKT_SIZE:
            return
        marker, roll, pitch, yaw, gx, gy, gz, target_roll, _ = \
            struct.unpack_from(IMU_PKT_FMT, data, 0)
        if marker != ord('I'):
            return

        now = time.monotonic()
        self._last_packet_time = now

        with self._lock:
            self._roll = roll
            self._rate = gx
            self._target_roll = target_roll

        udp_pkt = struct.pack(UDP_IMU_FMT, now, roll, pitch, yaw, gx, gy, gz, target_roll)

        try:
            self._udp.sendto(udp_pkt, self._unity_addr)
        except Exception:
            pass

    async def _send(self, data: bytes):
        if self._client and self._client.is_connected:
            try:
                await self._client.write_gatt_char(RX_CHAR_UUID, data, response=False)
            except Exception as e:
                log.warning("[BLE] Send failed: %s", e)

# rl helpers
def make_obs(roll, rate, target, angle_kp_norm, rate_kp_norm):
    return np.array([np.clip(roll / MAX_ANGLE_DEG, -1.0, 1.0), np.clip(rate / MAX_RATE_DPS,  -1.0, 1.0), np.clip(target / MAX_ANGLE_DEG, -1.0, 1.0), np.clip((roll - target) / MAX_ANGLE_DEG, -1.0, 1.0), angle_kp_norm, rate_kp_norm,], dtype=np.float32)

def compute_reward(errors, rates, prev_angle_kp, prev_rate_kp, cur_angle_kp, cur_rate_kp, crashed):
    if crashed:
        return CRASH_PENALTY
    errors = np.array(errors)
    rates = np.array(rates)
    # same logic as step rewards summed
    reward = -(W_TRACKING * np.mean(np.abs(errors)) + W_RATE * np.mean(np.abs(rates)))
    if np.mean(np.abs(errors)) < STABLE_DEG:
        reward += STABILITY_BONUS
    return float(reward)

def training_loop(drone: DroneInterface, stop_event: threading.Event, status_fn):
    log.info("[RL] Training thread started.")
    status_fn("STATUS:TRAINING")

    drone.claim_control()

    try:
        agent = SACAgent(obs_dim=6, action_dim=2, lr=1e-4)
        total_steps = 0
        best_reward = -float("inf")
        best_action = None
        exploitation_episodes = 0
        smoothed_quality = 50.0 # start at 50

        for episode in range(1, MAX_EPISODES + 1):
            if stop_event.is_set():
                break

            log.info(f"[RL] ------ Episode {episode} ------")
            sign = np.random.choice([-1, 1])
            target_deg = float(sign * np.random.uniform(10.0, TARGET_RANGE_DEG))
            target_stick = target_deg / TARGET_RANGE_DEG

            drone.send_safe_defaults()
            time.sleep(SETTLE_DURATION)

            imu = drone.read_imu()
            angle_kp_norm, rate_kp_norm = drone.normalise_gains()
            obs = make_obs(imu["roll_angle"], imu["roll_rate"], target_deg, angle_kp_norm, rate_kp_norm)

            if episode <= WARMUP_EPISODES:
                action = np.array([-1.0 + np.random.uniform(0, 0.3), -1.0 + np.random.uniform(0, 0.3)], dtype=np.float32)
            elif exploitation_episodes > 0 and best_action is not None:
                action = best_action
                exploitation_episodes -= 1
                log.info(f"[RL] --- Exploiting best gains ({exploitation_episodes} left) ---")
            else:
                action = agent.select_action(obs, deterministic=False)

            angle_kp, roll_p = drone.scale_action(action)
            stick_roll = target_stick

            prev_angle_kp = drone.current_angle_kp
            prev_rate_kp = drone.current_roll_p
            drone.send_command(stick_roll, angle_kp, roll_p)

            errors, rates = [], []
            crashed = False
            ep_start = time.time()

            while time.time() - ep_start < EPISODE_DURATION:
                if stop_event.is_set():
                    break
                if not drone.is_connected():
                    log.warning("[RL] BLE disconnected mid-episode, pausing...")
                    while not drone.is_connected() and not stop_event.is_set():
                        time.sleep(0.5)
                    if stop_event.is_set():
                        break
                    drone.claim_control()
                    ep_start = time.time()

                imu = drone.read_imu()
                roll = imu["roll_angle"]
                rate = imu["roll_rate"]
                error = roll - target_deg
                errors.append(error)
                rates.append(rate)

                if abs(roll) > 44.0:
                    log.warning(f"[RL] Safety cutoff! roll={roll:.1f}°")
                    crashed = True
                    break

                angle_kp_norm, rate_kp_norm = drone.normalise_gains()
                next_obs = make_obs(roll, rate, target_deg, angle_kp_norm, rate_kp_norm)
                tracking_cost = abs(error) / TARGET_RANGE_DEG
                step_reward = -(W_TRACKING * tracking_cost + W_RATE * abs(rate) / MAX_RATE_DPS)
                if abs(error) < STABLE_DEG:
                    step_reward += STABILITY_BONUS / (EPISODE_DURATION / IMU_LOOP_DT)
                agent.store(obs, action, step_reward, next_obs, done=False)
                obs = next_obs
                total_steps += 1

                if episode > WARMUP_EPISODES and len(agent.buffer) >= agent.batch_size:
                    for _ in range(UPDATES_PER_STEP):
                        metrics = agent.update()
                        if metrics and total_steps % 100 == 0:
                            log.info(f"  step={total_steps} "
                                     f"critic={metrics['critic_loss']:.4f} "
                                     f"actor={metrics['actor_loss']:.4f} "
                                     f"α={metrics['alpha']:.4f}")
                time.sleep(IMU_LOOP_DT)

            if not errors:
                ep_reward = 0.0
                mean_error = 0.0
            else:
                ep_reward = compute_reward(errors, rates, prev_angle_kp, prev_rate_kp, angle_kp, roll_p, crashed)
                mean_error = np.mean(np.abs(errors))

            quality = max(0.0, min(100.0, (ep_reward + 30.0) / 30.0 * 100.0))

            log.info(f"[RL] Quality score: {quality:.0f}/100")
            log.info(
                f"[RL] Episode {episode} | reward={ep_reward:.3f} | mean_err={mean_error:.2f}° | "
                f"target={target_deg:.1f}° | "
                f"angleKP={angle_kp:.3f}  P={roll_p:.4f} I={SAFE_ROLL_I:.4f} D={SAFE_ROLL_D:.4f} | "
                f"{'CRASH' if crashed else 'ok'}"
            )
            status_fn(f"STATUS:QUALITY:{quality:.0f}")

            if ep_reward > best_reward and episode > WARMUP_EPISODES:
                best_reward = ep_reward
                best_action = action.copy()
                exploitation_episodes = 3
                agent.save("checkpoints/sac_best.pt")
                log.info(f"[RL] ★ New best reward={best_reward:.3f} — saved")

            if episode % SAVE_EVERY == 0:
                agent.save(f"checkpoints/sac_ep{episode}.pt")

        log.info("[RL] Training complete.")

    except Exception as exc:
        log.error(f"[RL] Training error: {exc}", exc_info=True)
        status_fn(f"STATUS:ERROR:{exc}")

    finally:
        drone.send_safe_defaults()
        drone.release_control()
        status_fn("STATUS:IDLE")

# udp
def command_listener(drone: DroneInterface, shutdown_event: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", CMD_PORT))
    sock.settimeout(0.5)

    status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_status(msg: str):
        log.info(f"[CMD] -> Unity: {msg}")
        try:
            status_sock.sendto(msg.encode(), (UNITY_IP, STATUS_PORT))
        except Exception as e:
            log.warning(f"[CMD] Status send failed: {e}")

    train_thread: threading.Thread | None = None
    stop_event: threading.Event | None = None

    log.info(f"[CMD] Listening for Unity commands on port {CMD_PORT}")
    send_status("STATUS:IDLE")

    while not shutdown_event.is_set():
        try:
            data, _ = sock.recvfrom(256)
            cmd = data.decode().strip()
            log.info(f"[CMD] Received: '{cmd}'")

            if cmd == "START_TRAINING":
                if not drone.is_connected():
                    log.warning("[CMD] ESP32 not connected, refusing START_TRAINING")
                    send_status("STATUS:ERROR:ESP32 not connected")
                elif train_thread and train_thread.is_alive():
                    log.warning("[CMD] Training already running")
                    send_status("STATUS:TRAINING")
                else:
                    stop_event = threading.Event()
                    train_thread = threading.Thread(target=training_loop, args=(drone, stop_event, send_status), daemon=True)
                    train_thread.start()

            elif cmd == "STOP_TRAINING":
                if train_thread and train_thread.is_alive():
                    stop_event.set()
                    send_status("STATUS:STOPPED")
                else:
                    send_status("STATUS:IDLE")

            else:
                log.warning(f"[CMD] Unknown command: '{cmd}'")

        except socket.timeout:
            continue
        except Exception as e:
            log.error(f"[CMD] Listener error: {e}")

    sock.close()
    status_sock.close()

if __name__ == "__main__":
    log.info("╔════════════════════════════════════════════╗")
    log.info("║           PidraQRL - rl_bridge.py          ║")
    log.info("║    Copyright (c) 2025-2026 Hannes Göök     ║")
    log.info("║           MIT License - PidraQRL           ║")
    log.info("║   https://github.com/hannesgook/pidraqrl   ║")
    log.info("╚════════════════════════════════════════════╝")
    log.info("RL Bridge starting")

    log.info(f"IMU -> Unity UDP {UNITY_IP}:{UNITY_PORT}")
    log.info(f"Cmds <- Unity UDP 0.0.0.0:{CMD_PORT}")
    log.info(f"Status -> Unity UDP {UNITY_IP}:{STATUS_PORT}")

    selftest_packet()

    drone = DroneInterface()
    shutdown = threading.Event()

    threading.Thread(target=command_listener, args=(drone, shutdown), daemon=True).start()

    try:
        while True:
            time.sleep(1.0)
            imu = drone.read_imu()
            status = "CONNECTED" if drone.is_connected() else "scanning..."
    except KeyboardInterrupt:
        log.info("Shutting down...")
        shutdown.set()