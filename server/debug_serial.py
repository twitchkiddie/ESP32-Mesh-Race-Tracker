"""Quick debug script to test serial frame parsing from gateway."""
import serial, time, sys

sys.stdout.reconfigure(errors='replace')

from serial_bridge import (crc16, FrameParser, mac_to_str,
                            HEADER_FMT, GPS_BODY_FMT, HB_BODY_FMT, GW_HB_FMT,
                            MSG_GPS, MSG_HEARTBEAT, MSG_GW_STATUS)

parser = FrameParser()
frames_rx = 0
bytes_rx = 0

print('Opening COM7...', flush=True)

s = serial.Serial()
s.port = 'COM7'
s.baudrate = 115200
s.timeout = 0.5
s.dsrdtr = False
s.rtscts = False
s.open()
s.setDTR(False)
s.setRTS(False)

print('Connected. Listening for 25s (board may reset on port open)...', flush=True)
start = time.time()

while time.time() - start < 25:
    raw = s.read(256)
    if not raw:
        continue
    bytes_rx += len(raw)
    for byte in raw:
        payload = parser.feed(byte)
        if payload is not None:
            frames_rx += 1
            t = payload[0]
            elapsed = time.time() - start
            print(f'[Frame #{frames_rx} t={elapsed:.1f}s] type=0x{t:02X} len={len(payload)}', flush=True)
            if t == MSG_GW_STATUS and len(payload) >= GW_HB_FMT.size:
                _, nc, up, heap = GW_HB_FMT.unpack_from(payload)
                print(f'  GW Heartbeat: nodes={nc} uptime={up}s heap={heap}', flush=True)
            elif t in (MSG_GPS, MSG_HEARTBEAT) and len(payload) >= HEADER_FMT.size:
                hdr = HEADER_FMT.unpack_from(payload)
                mac = mac_to_str(hdr[4])
                print(f'  Header: type=0x{hdr[0]:02X} hop={hdr[1]} id={hdr[2]} seq={hdr[3]} mac={mac}', flush=True)

print(f'Done. bytes_rx={bytes_rx} frames_rx={frames_rx}', flush=True)
s.close()
