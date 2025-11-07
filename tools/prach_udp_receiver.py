#!/usr/bin/env python3
"""
Simple PRACH UDP receiver for local testing.

Listens on 0.0.0.0:5678 and collects 40 packets (10 cycles x 4 antennas).
Prints a progress message for each received packet like "1. paket alındı".

It will try to parse a packed RU header if present. The RU header layout is:
  uint32_t frame;
  uint32_t slot;
  uint8_t  antenna;
  uint8_t  prachOccasion;
  uint16_t N_ZC;
  uint16_t data_len;  # count of int16 IQ values

Header is expected in network byte order when present. If no header is
present, the script treats the entire UDP payload as IQ data.
"""
import socket
import struct
import sys
import os

LISTEN_IP = '0.0.0.0'
LISTEN_PORT = 5678
EXPECTED_PACKETS = 40  # 10 cycles * 4 antennas

# directory to save received packets (expand ~ to home)
SAVE_DIR = os.path.expanduser('~/prach_udp_rec')


def try_parse_header(data: bytes):
    # header is packed and 14 bytes long (4+4+1+1+2+2)
    if len(data) < 14:
        return None
    try:
        # '!II2B2H' -> network byte order: uint32, uint32, 2xuint8, 2xuint16
        frame, slot, antenna, prachOcc, N_ZC, data_len = struct.unpack('!II2B2H', data[:14])
        return {
            'frame': frame,
            'slot': slot,
            'antenna': antenna,
            'prachOcc': prachOcc,
            'N_ZC': N_ZC,
            'data_len': data_len,
            'payload': data[14:]
        }
    except struct.error:
        return None

def main():
    # ensure save directory exists
    if not os.path.exists(SAVE_DIR):
        try:
            os.makedirs(SAVE_DIR, exist_ok=True)
            print(f"Created save directory: {SAVE_DIR}")
        except Exception as e:
            print(f"Failed to create save directory {SAVE_DIR}: {e}")
            return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"Listening on {LISTEN_IP}:{LISTEN_PORT}, expecting {EXPECTED_PACKETS} packets")

    count = 0
    try:
        while count < EXPECTED_PACKETS:
            data, addr = sock.recvfrom(65536)
            count += 1
            hdr = try_parse_header(data)
            if hdr is not None:
                payload = hdr['payload']
                payload_len = len(payload)
                print(f"{count}. paket alındı - from {addr} | ant={hdr['antenna']} occ={hdr['prachOcc']} frame={hdr['frame']} slot={hdr['slot']} N_ZC={hdr['N_ZC']} data_len={hdr['data_len']} payload_bytes={payload_len}")
                # save payload to file for inspection
                fname = os.path.join(SAVE_DIR, f"prach_recv_{count:03d}_ant{hdr['antenna']:02d}_fr{hdr['frame']:04d}_sl{hdr['slot']:03d}.bin")
                with open(fname, 'wb') as f:
                    f.write(payload)
                print(f"  -> saved payload to {fname}")
                # try to preview first few int16 IQ pairs (if payload length sufficient)
                if payload_len >= 4:
                    # interpret payload as little-endian signed int16 (common on x86)
                    nvals = payload_len // 2
                    try:
                        vals = struct.unpack('<%dh' % nvals, payload)
                    except struct.error:
                        vals = struct.unpack('%dh' % nvals, payload)

                    # basic statistics to detect all-zero payloads vs real samples
                    nonzero = sum(1 for v in vals if v != 0)
                    absmax = max(abs(v) for v in vals) if vals else 0
                    maxv = max(vals) if vals else 0
                    minv = min(vals) if vals else 0
                    pairs = [(vals[i], vals[i+1]) for i in range(0, min(len(vals), 12), 2)]
                    print(f"  preview (I,Q) samples: {pairs}")
                    print(f"  stats: samples={nvals}, nonzero={nonzero}, max={maxv}, min={minv}, absmax={absmax}")
            else:
                print(f"{count}. paket alındı - from {addr} | raw bytes={len(data)} (no header)")
                fname = os.path.join(SAVE_DIR, f"prach_recv_{count:03d}_raw.bin")
                with open(fname, 'wb') as f:
                    f.write(data)
                print(f"  -> saved raw packet to {fname}")
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        sock.close()
        print(f"Received {count} packets, exiting")

if __name__ == '__main__':
    main()
