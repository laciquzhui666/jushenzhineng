"""Tiny dependency-free PNG writer for PyBullet camera frames."""
import struct
import zlib

import numpy as np


def write_png(path, arr):
    arr = np.asarray(arr, dtype=np.uint8)
    if arr.ndim == 2:
        arr = np.repeat(arr[:, :, None], 3, axis=2)
    h, w, c = arr.shape
    color_type = 6 if c == 4 else 2
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw += arr[y].tobytes()

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, color_type, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)
