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
import time
import argparse

try:
    import numpy as np
except Exception as e:
    print('Error: numpy is required for this script. Install with: pip install numpy')
    raise

LISTEN_IP = '0.0.0.0'
LISTEN_PORT = 5678
EXPECTED_PACKETS = 40  # default

# directory to save received packets (expand ~ to home)
SAVE_DIR = os.path.expanduser('~/prach_udp_rec')


def try_parse_header(data: bytes):
    # Expect a 4-byte magic prefix followed by a 14-byte header
    # wire layout: 'PRCH' + 14-byte header ('!II2B2H') + payload
    if len(data) < 18:
        return None
    try:
        if data[:4] != b'PRCH':
            return None
        # '!II2B2H' -> network byte order: uint32, uint32, 2xuint8, 2xuint16
        frame, slot, antenna, prachOcc, N_ZC, data_len = struct.unpack('!II2B2H', data[4:18])
        # Basic plausibility checks to avoid mis-parsing raw IQ as header
        remaining = len(data) - 18
        # data_len is interpreted as number of int16 values in payload
        expected_bytes = data_len * 2
        if antenna > 16:
            return None
        if N_ZC not in (139, 839):
            return None
        # allow some tolerance: expected_bytes should not exceed remaining
        if expected_bytes > remaining or expected_bytes == 0 or expected_bytes > 65536:
            return None

        return {
            'frame': frame,
            'slot': slot,
            'antenna': antenna,
            'prachOcc': prachOcc,
            'N_ZC': N_ZC,
            'data_len': data_len,
            'payload': data[18:18+expected_bytes]
        }
    except struct.error:
        return None


def payload_to_complex(payload: bytes):
    # Interpret payload as little-endian int16 I,Q interleaved
    if len(payload) < 4:
        return np.array([], dtype=np.complex64)
    # number of int16 values available
    n_int16 = len(payload) // 2
    # number of complex samples (pairs of int16 -> I,Q)
    n_pairs = n_int16 // 2
    if n_pairs == 0:
        return np.array([], dtype=np.complex64)
    bytes_to_use = n_pairs * 2 * 2  # n_pairs * (2 int16) * (2 bytes per int16)
    vals = np.frombuffer(payload[:bytes_to_use], dtype='<i2').astype(np.float32)
    vals = vals.reshape(-1, 2)
    cplx = vals[:, 0] + 1j * vals[:, 1]
    return cplx


def cal_data_py(data, fax, t):
    # data: (N,4) complex numpy array
    # fax: frequency axis (length N)
    # t: time samples (length N)
    N = data.shape[0]
    df1 = np.fft.fftshift(np.fft.fft(data[:, 0]))
    df2 = np.fft.fftshift(np.fft.fft(data[:, 1]))
    df3 = np.fft.fftshift(np.fft.fft(data[:, 2]))
    df4 = np.fft.fftshift(np.fft.fft(data[:, 3]))

    pf1 = int(np.argmax(np.abs(df1)))
    pf2 = int(np.argmax(np.abs(df2)))
    pf3 = int(np.argmax(np.abs(df3)))
    pf4 = int(np.argmax(np.abs(df4)))

    mv1 = df1[pf1]
    mv2 = df2[pf2]
    mv3 = df3[pf3]
    mv4 = df4[pf4]

    eps = 1e-12
    mv2s = mv2 if abs(mv2) > eps else (mv2 + eps)
    mv3s = mv3 if abs(mv3) > eps else (mv3 + eps)
    mv4s = mv4 if abs(mv4) > eps else (mv4 + eps)

    cal2 = (mv1 / mv2s) * np.exp(1j * 2.0 * np.pi * t * (fax[pf1] - fax[pf2]))
    cal3 = (mv1 / mv3s) * np.exp(1j * 2.0 * np.pi * t * (fax[pf1] - fax[pf3]))
    cal4 = (mv1 / mv4s) * np.exp(1j * 2.0 * np.pi * t * (fax[pf1] - fax[pf4]))

    datac = np.empty_like(data, dtype=np.complex128)
    datac[:, 0] = data[:, 0]
    datac[:, 1] = cal2 * data[:, 1]
    datac[:, 2] = cal3 * data[:, 2]
    datac[:, 3] = cal4 * data[:, 3]

    return datac, cal2, cal3, cal4


def mus_est_v3_py(data, fc=2.6e9, ax_points=100):
    # data: (N,M) complex numpy array
    df = np.fft.fftshift(np.fft.fft(data, axis=0), axes=0)  # (N,M)
    lam = 3e8 / fc
    # antenna positions as in MATLAB (mm -> m)
    el = np.array([0.0, 47.5, 47.5 + 57.5, 2 * 47.5 + 57.5]) / 1000.0
    el = el.reshape(-1, 1)

    ax = np.linspace(-90.0, 90.0, ax_points)
    axr = np.deg2rad(ax)
    # faz: M x ax_points
    faz = np.exp(1j * 2.0 * np.pi * (el / lam) * np.sin(axr))

    # K = faz' * faz  => (ax_points x ax_points)
    K = np.conj(faz).T @ faz
    eigvals = np.linalg.eigvals(K)
    meig = np.max(eigvals) / 100.0

    rhs = np.conj(faz).T @ df.T  # (ax_points, N)
    lhs = K + np.eye(K.shape[0]) * meig
    df2_mat = np.linalg.solve(lhs, rhs)
    power = np.sum(np.abs(df2_mat) ** 2, axis=1)
    if np.max(power) > 0:
        power = power / np.max(power)
    peak_idx = int(np.argmax(power))
    th = ax[peak_idx]
    return power, float(th), ax
def main():
    parser = argparse.ArgumentParser(description='PRACH UDP receiver with auto-once calibration and MATLAB-port DoA')
    parser.add_argument('--listen-ip', default=LISTEN_IP)
    parser.add_argument('--port', type=int, default=LISTEN_PORT)
    parser.add_argument('--mode', choices=['auto'], default='auto', help='Operation mode (auto: calibrate once then run)')
    parser.add_argument('--calibration-packets', type=int, default=4, help='Number of unique groups used for calibration')
    parser.add_argument('--cal-file', default='./cal.npz', help='File to save computed calibration (npz)')
    parser.add_argument('--fs', type=float, default=(245.76e6 / 200.0), help='Sampling rate (Hz) for fax/t generation')
    parser.add_argument('--fc', type=float, default=2.6e9, help='Carrier frequency (Hz) for DoA estimator')
    parser.add_argument('--ignore-frame', action='store_true', help='Group packets by (slot,occ) only, ignore frame field')
    parser.add_argument('--expected-packets', type=int, default=EXPECTED_PACKETS, help='Number of groups to process after calibration')
    args = parser.parse_args()

    if not os.path.exists(args.cal_file) and not os.path.exists(SAVE_DIR):
        try:
            os.makedirs(SAVE_DIR, exist_ok=True)
        except Exception as e:
            print(f"Failed to create save directory {SAVE_DIR}: {e}")
            return

    listen_addr = (args.listen_ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(listen_addr)
    print(f"Listening on {listen_addr[0]}:{listen_addr[1]} (mode={args.mode})")

    fs = float(args.fs)
    Ts = 1.0 / fs

    groups = {}  # group_key -> {ant: complex_array}
    calib_list_2 = []
    calib_list_3 = []
    calib_list_4 = []
    calib_done = False
    calib_count = 0
    processed = 0

    try:
        while processed < args.expected_packets:
            data, addr = sock.recvfrom(65536)
            hdr = try_parse_header(data)
            if hdr is None:
                # No valid header detected: treat entire UDP payload as raw IQ
                payload = data
                payload_len = len(payload)
                now = time.time()
                print(f"pkt from {addr} NO-HEADER bytes={payload_len} -- saving raw payload")
                # Save raw packet for offline inspection
                fname = os.path.join(SAVE_DIR, f"prach_nohdr_{addr[1]}_{int(time.time()*1000)}.bin")
                try:
                    with open(fname, 'wb') as f:
                        f.write(payload)
                except Exception as e:
                    print(f"Failed to save raw no-header packet: {e}")
                # skip grouping logic since we lack antenna/frame metadata
                continue

            payload = hdr['payload']
            payload_len = len(payload)
            now = time.time()
            # include header-declared data_len (if present) to aid debugging
            data_len_field = hdr.get('data_len', None)
            print(f"pkt from {addr} ant={hdr['antenna']} fr={hdr['frame']} sl={hdr['slot']} occ={hdr['prachOcc']} bytes={payload_len} data_len_header={data_len_field}")

            # convert to complex
            cplx = payload_to_complex(payload)
            # map antenna index to 0..3
            ant = int(hdr['antenna'])
            # prefer 0-based if values 0..3 are used; fall back to 1-based mapping
            if 0 <= ant <= 3:
                ant_idx = ant
            elif 1 <= ant <= 4:
                ant_idx = ant - 1
            else:
                # unexpected antenna id: reduce modulo 4 and warn
                ant_idx = ant % 4
                print(f"Warning: unexpected antenna id {ant}; mapped to {ant_idx}")

            if args.ignore_frame:
                group_key = (int(hdr['slot']), int(hdr['prachOcc']))
            else:
                group_key = (int(hdr['frame']), int(hdr['slot']), int(hdr['prachOcc']))
            if group_key not in groups:
                groups[group_key] = {}
            groups[group_key][ant_idx] = cplx

            # save raw payload for inspection
            fname = os.path.join(SAVE_DIR, f"prach_recv_fr{hdr['frame']}_sl{hdr['slot']}_occ{hdr['prachOcc']}_ant{ant_idx}.bin")
            with open(fname, 'wb') as f:
                f.write(payload)

            # if we are still calibrating, wait until we have all 4 antennas for a group
            if (not calib_done) and (len(groups[group_key]) == 4):
                # build data matrix Nx4 ordered by antenna index 0..3
                ants_sorted = sorted(groups[group_key].keys())
                cols = [groups[group_key].get(i, np.array([], dtype=np.complex128)) for i in range(4)]
                lengths = [c.size for c in cols]
                if len(set(lengths)) != 1 or lengths[0] == 0:
                    print(f"Skipping group {group_key}: unequal payload lengths {lengths}")
                    del groups[group_key]
                    continue
                N = lengths[0]
                data_mat = np.column_stack(cols)
                # t and fax
                t = np.arange(N) * Ts
                fax = np.fft.fftshift(np.fft.fftfreq(N, d=Ts))

                datac, cal2, cal3, cal4 = cal_data_py(data_mat, fax, t)
                calib_list_2.append(cal2)
                calib_list_3.append(cal3)
                calib_list_4.append(cal4)
                calib_count += 1
                print(f"[{time.strftime('%H:%M:%S')}] Collected calibration group {calib_count}/{args.calibration_packets} for {group_key}")
                del groups[group_key]

                if calib_count >= args.calibration_packets:
                    # average calibrations
                    avg2 = np.mean(np.stack(calib_list_2, axis=0), axis=0)
                    avg3 = np.mean(np.stack(calib_list_3, axis=0), axis=0)
                    avg4 = np.mean(np.stack(calib_list_4, axis=0), axis=0)
                    np.savez(args.cal_file, cal2=avg2, cal3=avg3, cal4=avg4)
                    print(f"[{time.strftime('%H:%M:%S')}] Calibration complete. Saved to {args.cal_file}")
                    calib_done = True
                    # continue to next loop to start processing groups
                    continue

            # If calibration done, process groups as they become complete
            if calib_done and (len(groups[group_key]) == 4):
                cols = [groups[group_key].get(i, np.array([], dtype=np.complex128)) for i in range(4)]
                lengths = [c.size for c in cols]
                if len(set(lengths)) != 1 or lengths[0] == 0:
                    print(f"Skipping group {group_key}: unequal payload lengths {lengths}")
                    del groups[group_key]
                    continue
                N = lengths[0]
                data_mat = np.column_stack(cols)
                # load calibration file
                npz = np.load(args.cal_file)
                avg2 = npz['cal2']
                avg3 = npz['cal3']
                avg4 = npz['cal4']
                # apply calibration
                datac = np.empty_like(data_mat, dtype=np.complex128)
                datac[:, 0] = data_mat[:, 0]
                datac[:, 1] = avg2 * data_mat[:, 1]
                datac[:, 2] = avg3 * data_mat[:, 2]
                datac[:, 3] = avg4 * data_mat[:, 3]
                # run DoA estimator
                power, th_deg, ax = mus_est_v3_py(datac, fc=args.fc)
                print(f"[{time.strftime('%H:%M:%S')}] DoA estimate for group {group_key}: {th_deg:.2f} deg")
                processed += 1
                del groups[group_key]

    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        sock.close()


if __name__ == '__main__':
    main()
