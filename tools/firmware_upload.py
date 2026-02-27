#!/usr/bin/env python3
"""
ESP32 Mesh V2 - Firmware Upload Tool
Sends a compiled .bin firmware to a tracker node over the ESP-NOW mesh
via the gateway's serial port. Uses stop-and-wait ARQ for reliability.

Usage:
    # Stop mesh_server.py first (it holds COM7), then:
    python tools/firmware_upload.py .pio/build/esp32dev/firmware.bin --port COM7
    python tools/firmware_upload.py firmware.bin --port COM7 --timeout 2.0

The tool:
    1. Sends OTA_CTRL_START with firmware size + MD5
    2. Sends OTA_DATA chunks one at a time, waits for ACK
    3. Retransmits on timeout (up to --retries times)
    4. Prints progress; target auto-reboots after last chunk is verified
"""

import argparse
import hashlib
import struct
import sys
import time
import os

import serial

# ---------------------------------------------------------------------------
# Protocol constants (must match protocol.h / config.h)
# ---------------------------------------------------------------------------
SYNC1 = 0xAA
SYNC2 = 0x55
MAX_FRAME = 256

MSG_OTA_CTRL = 0x20
MSG_OTA_DATA = 0x21
MSG_ACK      = 0x30

OTA_ACTION_START = 0
OTA_ACTION_ABORT = 1

CHUNK_SIZE = 200   # Must match OTA_CHUNK_SIZE in config.h

# PacketHeader: msg_type(B) hop_count(B) device_id(B) seq_num(B) src_mac(6s)
HEADER_STRUCT = struct.Struct('<BBBB6s')

# OTAControlMessage body (after 10-byte header):
# action(B) firmware_size(I) total_chunks(H) hash(16s) = 1+4+2+16 = 23 bytes
OTA_CTRL_BODY = struct.Struct('<BIH16s')

# OTADataMessage variable header (after 10-byte header):
# chunk_index(H) chunk_len(B)
OTA_DATA_HDR  = struct.Struct('<HB')

# AckMessage body (after 10-byte header):
# ack_msg_type(B) ack_seq_num(B) status(B) = 3 bytes
ACK_BODY = struct.Struct('<BBB')

NULL_MAC = bytes(6)  # Gateway fills in its own MAC; use zeros here


# ---------------------------------------------------------------------------
# CRC16-CCITT (init=0x0000, poly=0x1021) -- matches SerialProto::crc16
# ---------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Frame framing / parsing
# ---------------------------------------------------------------------------
def build_frame(payload: bytes) -> bytes:
    crc = crc16(payload)
    hdr = bytes([SYNC1, SYNC2, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    return hdr + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_ota_ctrl(fw_data: bytes, total_chunks: int, seq: int) -> bytes:
    md5 = hashlib.md5(fw_data).digest()
    header = HEADER_STRUCT.pack(MSG_OTA_CTRL, 0, 0, seq & 0xFF, NULL_MAC)
    body   = OTA_CTRL_BODY.pack(OTA_ACTION_START, len(fw_data), total_chunks, md5)
    return build_frame(header + body)


def build_ota_data(chunk_index: int, chunk: bytes, seq: int) -> bytes:
    header = HEADER_STRUCT.pack(MSG_OTA_DATA, 0, 0, seq & 0xFF, NULL_MAC)
    body   = OTA_DATA_HDR.pack(chunk_index, len(chunk)) + chunk
    return build_frame(header + body)


# ---------------------------------------------------------------------------
# Frame reader (state machine, reads from serial port)
# ---------------------------------------------------------------------------
class FrameReader:
    def __init__(self):
        self._state = 'S1'
        self._buf = bytearray()
        self._elen = 0
        self._crc_lo = 0

    def feed(self, b: int):
        """Feed one byte; returns payload bytes if frame complete, else None."""
        if self._state == 'S1':
            if b == SYNC1: self._state = 'S2'
        elif self._state == 'S2':
            self._state = 'LL' if b == SYNC2 else 'S1'
        elif self._state == 'LL':
            self._elen = b; self._state = 'LH'
        elif self._state == 'LH':
            self._elen |= (b << 8)
            if 0 < self._elen <= MAX_FRAME:
                self._buf = bytearray(); self._state = 'PL'
            else:
                self._state = 'S1'
        elif self._state == 'PL':
            self._buf.append(b)
            if len(self._buf) >= self._elen: self._state = 'CL'
        elif self._state == 'CL':
            self._crc_lo = b; self._state = 'CH'
        elif self._state == 'CH':
            rx_crc = self._crc_lo | (b << 8)
            payload = bytes(self._buf)
            self._state = 'S1'
            return payload if crc16(payload) == rx_crc else None
        return None


# ---------------------------------------------------------------------------
# Wait for an ACK frame matching a given seq_num
# ---------------------------------------------------------------------------
GW_HEARTBEAT = 0xFF   # Gateway status frame marker (sent every 5s)


def wait_for_gateway_ready(ser: serial.Serial, timeout: float = 12.0) -> bool:
    """
    Wait until the gateway sends a framed packet of any kind.
    Returns True when we see a valid frame (gateway is alive and in GATEWAY mode).
    """
    print(f"Waiting for gateway (up to {timeout:.0f}s)...", end=' ', flush=True)
    reader = FrameReader()
    deadline = time.time() + timeout
    while time.time() < deadline:
        ser.timeout = max(0.05, deadline - time.time())
        chunk = ser.read(256)
        for b in chunk:
            payload = reader.feed(b)
            if payload is not None and len(payload) >= 1:
                if payload[0] == GW_HEARTBEAT:
                    print("gateway heartbeat received OK")
                elif len(payload) >= HEADER_STRUCT.size:
                    hdr = HEADER_STRUCT.unpack_from(payload, 0)
                    print(f"frame received (type=0x{hdr[0]:02X}) OK")
                else:
                    print(f"frame received ({len(payload)}B) OK")
                return True
    print("timeout - check gateway is in GATEWAY mode and COM port is correct")
    return False


def wait_for_ack(ser: serial.Serial, seq: int, timeout: float, verbose: bool = False) -> bool:
    """
    Read bytes from serial until we get an ACK for `seq`, or timeout.
    Returns True if ACK received with status=0 (OK).
    """
    reader = FrameReader()
    deadline = time.time() + timeout
    while time.time() < deadline:
        ser.timeout = max(0.01, deadline - time.time())
        chunk = ser.read(256)
        for b in chunk:
            payload = reader.feed(b)
            if payload is None:
                continue
            if verbose and len(payload) >= 1:
                t = payload[0] if len(payload) >= 1 else 0
                print(f"\n  [DBG] frame len={len(payload)} type=0x{t:02X}", end='', flush=True)
            if len(payload) < HEADER_STRUCT.size + ACK_BODY.size:
                continue
            hdr = HEADER_STRUCT.unpack_from(payload, 0)
            msg_type = hdr[0]
            if msg_type != MSG_ACK:
                continue
            _, ack_seq, status = ACK_BODY.unpack_from(payload, HEADER_STRUCT.size)
            if verbose:
                print(f"\n  [ACK] ack_seq={ack_seq} expected={seq & 0xFF} status={status}", flush=True)
            if ack_seq == (seq & 0xFF):
                return status == 0
    return False


# ---------------------------------------------------------------------------
# Main upload logic
# ---------------------------------------------------------------------------
def upload(args):
    if not os.path.isfile(args.firmware):
        print(f"Error: firmware file not found: {args.firmware}")
        sys.exit(1)

    with open(args.firmware, 'rb') as f:
        fw = f.read()

    fw_size    = len(fw)
    md5_hex    = hashlib.md5(fw).hexdigest()
    chunks     = [fw[i:i+CHUNK_SIZE] for i in range(0, fw_size, CHUNK_SIZE)]
    n_chunks   = len(chunks)

    print(f"Firmware : {args.firmware}")
    print(f"Size     : {fw_size:,} bytes ({fw_size/1024:.1f} KB)")
    print(f"MD5      : {md5_hex}")
    print(f"Chunks   : {n_chunks}  x  {CHUNK_SIZE} bytes")
    print(f"Port     : {args.port}  @  {args.baud}")
    print(f"ETA      : ~{n_chunks * args.timeout:.0f}s worst-case, ~{n_chunks * 0.1:.0f}s typical")
    print()

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout, dsrdtr=False, rtscts=False)
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(1.0)  # Let ESP32 settle (opening port can briefly reset it)

    # --- Confirm gateway is alive and in GATEWAY mode ---
    if not wait_for_gateway_ready(ser, timeout=12.0):
        print("\nGateway not responding. Check:")
        print("  1. COM port is correct (--port COM7)")
        print("  2. Gateway has role=GATEWAY in NVS (serial: 'role gateway' + reboot)")
        print("  3. No other process holding the serial port")
        ser.close()
        sys.exit(1)

    seq = 0   # Global monotonic seq counter -- always increments per send

    # --- OTA loop with auto-restart on failure ---
    # The tracker's OTA handler can abort mid-transfer (Update.write from WiFi callback
    # context is unreliable on ESP32). Auto-restart re-sends OTA_CTRL and retries from
    # chunk 0. Since each restart attempts more chunks, eventually one run goes all the
    # way through.
    MAX_RESTARTS  = 30
    total_retries = 0
    t_start       = time.time()
    completed     = False

    for run in range(1, MAX_RESTARTS + 1):
        # --- (Re-)send OTA_CTRL ---
        if run == 1:
            print("Sending OTA start...", end=' ', flush=True)
        else:
            print(f"\n[restart {run}] Tracker aborted - re-sending OTA start...", end=' ', flush=True)

        ctrl_ok = False
        for attempt in range(8):
            ser.write(build_ota_ctrl(fw, n_chunks, seq))
            verbose = (attempt == 0 and run == 1)
            if wait_for_ack(ser, seq, args.timeout, verbose=verbose):
                seq += 1
                ctrl_ok = True
                print("ACK received OK")
                break
            seq += 1
            print(f"timeout (attempt {attempt+1})", end=' ', flush=True)

        if not ctrl_ok:
            print("\nNo ACK for OTA start - is the tracker powered on and in range?")
            ser.close()
            sys.exit(1)

        print()

        # --- Send chunks (from 0 on each restart) ---
        i           = 0
        run_ok      = True

        while i < n_chunks:
            chunk = chunks[i]
            sent  = False

            for attempt in range(args.retries + 1):
                if attempt > 0:
                    total_retries += 1
                ser.write(build_ota_data(i, chunk, seq))
                if wait_for_ack(ser, seq, args.timeout):
                    seq += 1
                    sent = True
                    break
                seq += 1  # Fresh seq bypasses relay dedup on retry

            if not sent:
                # Tracker likely aborted -- will restart OTA
                run_ok = False
                break

            i += 1

            # Progress bar
            elapsed = time.time() - t_start
            rate    = i / elapsed if elapsed > 0.01 else 1
            eta     = (n_chunks - i) / rate
            pct     = (i * 100) // n_chunks
            bar     = '#' * (pct // 5) + '.' * (20 - pct // 5)
            sys.stdout.write(
                f"\r  [{bar}] {pct:3d}%  {i}/{n_chunks}  {rate:.0f} chunk/s  ETA {eta:.0f}s  retries={total_retries}  run={run}  "
            )
            sys.stdout.flush()

        if run_ok:
            completed = True
            break

    elapsed = time.time() - t_start
    if not completed:
        print(f"\n\nFailed after {MAX_RESTARTS} restart attempts. Check signal strength.")
        ser.close()
        sys.exit(1)

    print(f"\n\nUpload complete: {n_chunks} chunks in {elapsed:.1f}s  ({n_chunks*CHUNK_SIZE/elapsed/1024:.0f} KB/s)")
    print(f"Total retries: {total_retries}  Restarts: {run - 1}")
    print()
    print("Target is verifying MD5 and will reboot if successful.")
    print("Watch for it to reappear on the dashboard (typically ~5s).")

    ser.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Upload firmware to ESP32 tracker via ESP-NOW mesh OTA'
    )
    parser.add_argument('firmware',
                        help='Path to firmware .bin (e.g. .pio/build/esp32dev/firmware.bin)')
    parser.add_argument('--port', default='COM7',
                        help='Gateway serial port (default: COM7)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Serial baud rate (default: 115200)')
    parser.add_argument('--timeout', type=float, default=1.5,
                        help='ACK timeout per chunk in seconds (default: 1.5)')
    parser.add_argument('--retries', type=int, default=5,
                        help='Max retransmit attempts per chunk (default: 5)')
    parser.add_argument('--verbose', action='store_true',
                        help='Print all received frames for debugging')
    args = parser.parse_args()
    upload(args)


if __name__ == '__main__':
    main()
