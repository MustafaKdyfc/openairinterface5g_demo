#!/usr/bin/env python3
"""
Simple test sender: sends one UDP packet with RU header (network byte order)
and a known int16 pattern payload (278 int16 values for N_ZC=139).
"""
import socket, struct

dst = ('127.0.0.1', 5678)
frame = 1234
slot = 56
antenna = 1
prachOcc = 0
N_ZC = 139
# payload length in bytes (278 int16 values)
data_len_bytes = N_ZC * 2 * 2  # N_ZC * 2 samples * 2 bytes/sample = 556
# but many implementations use data_len as count of int16 values; try bytes here

# build header in network byte order
hdr = struct.pack('!II2B2H', frame, slot, antenna, prachOcc, N_ZC, data_len_bytes)

# build payload: 278 int16 values with a recognizable pattern
vals = [((i % 127) - 63) for i in range(N_ZC * 2)]
payload = struct.pack('<%dh' % (N_ZC * 2), *vals)

pkt = hdr + payload

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(pkt, dst)
print('sent', len(pkt))
