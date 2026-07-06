#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""タイルデータセットから低解像度の全域地図を生成する(オフライン版)。

通常は global_lowres_map_server ノードが起動時に自動生成するため不要だが、
地図を目視確認したい・stock nav2_map_server で配信したい場合に使う。
出力は map_server 互換の PGM + YAML。

  make_lowres_map.py --tileset map_tiles/tileset.yaml --factor 4 --out lowres

ノード実装 (downsampler.cpp) と同じ「占有優先 max-pool」で縮小する。
"""

import argparse
import pathlib
import re
import sys

import numpy as np
import yaml

UNKNOWN, FREE, OCCUPIED = -1, 0, 100
PGM_VALUE = {OCCUPIED: 0, FREE: 254, UNKNOWN: 205}
TILE_RE = re.compile(r'^tile_(-?\d+)_(-?\d+)\.pgm$')


def read_pgm_p5(path):
    data = path.read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i] != ord('\n'):
                i += 1
            continue
        s = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[s:i])
    i += 1
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or maxval > 255:
        raise ValueError(f'unsupported PGM: {path}')
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return img


def pgm_to_trinary(img, negate, occ_th, free_th):
    v = img.astype(np.float64) / 255.0
    occ = v if negate else 1.0 - v
    out = np.full(img.shape, UNKNOWN, dtype=np.int8)
    out[occ > occ_th] = OCCUPIED
    out[occ < free_th] = FREE
    return np.flipud(out)  # 行0=下端に揃える


def downsample_occupancy_priority(tile, f):
    """占有優先 max-pool。tile は行0=下端の {-1,0,100}。"""
    h, w = tile.shape
    lh, lw = h // f, w // f
    out = np.full((lh, lw), UNKNOWN, dtype=np.int8)
    for r in range(lh):
        for c in range(lw):
            block = tile[r * f:(r + 1) * f, c * f:(c + 1) * f]
            if np.any(block == OCCUPIED):
                out[r, c] = OCCUPIED
            elif np.any(block == FREE):
                out[r, c] = FREE
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--tileset', required=True, help='tileset.yaml')
    ap.add_argument('--factor', type=int, default=4, help='低解像度化係数')
    ap.add_argument('--out', required=True, help='出力プレフィックス(.pgm/.yaml)')
    args = ap.parse_args()

    ts_path = pathlib.Path(args.tileset)
    with open(ts_path) as f:
        ts = yaml.safe_load(f)
    resolution = float(ts['resolution'])
    tile_cells = int(ts['tile_size_cells'])
    ox, oy = float(ts['origin'][0]), float(ts['origin'][1])
    negate = bool(int(ts.get('negate', 0)))
    occ_th = float(ts.get('occupied_thresh', 0.65))
    free_th = float(ts.get('free_thresh', 0.196))
    f = args.factor
    if f <= 0 or tile_cells % f != 0:
        sys.exit(f'error: factor {f} must be > 0 and divide tile_size_cells {tile_cells}')

    tiles_dir = ts_path.parent / 'tiles'
    present = {}
    for p in tiles_dir.glob('tile_*.pgm'):
        m = TILE_RE.match(p.name)
        if m:
            present[(int(m.group(1)), int(m.group(2)))] = p
    if not present:
        sys.exit(f'error: no tiles in {tiles_dir}')

    xs = [k[0] for k in present]
    ys = [k[1] for k in present]
    min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
    lo_tile = tile_cells // f
    W = (max_x - min_x + 1) * lo_tile
    H = (max_y - min_y + 1) * lo_tile
    canvas = np.full((H, W), UNKNOWN, dtype=np.int8)

    for (tx, ty), path in present.items():
        tri = pgm_to_trinary(read_pgm_p5(path), negate, occ_th, free_th)
        lo = downsample_occupancy_priority(tri, f)
        r0 = (ty - min_y) * lo_tile
        c0 = (tx - min_x) * lo_tile
        canvas[r0:r0 + lo_tile, c0:c0 + lo_tile] = lo

    lut = np.full(256, PGM_VALUE[UNKNOWN], dtype=np.uint8)
    lut[FREE] = PGM_VALUE[FREE]
    lut[OCCUPIED] = PGM_VALUE[OCCUPIED]
    pgm = lut[canvas.astype(np.uint8)]

    out = pathlib.Path(args.out)
    pgm_path = out.with_suffix('.pgm')
    with open(pgm_path, 'wb') as fp:
        fp.write(f'P5\n{W} {H}\n255\n'.encode())
        fp.write(np.flipud(pgm).astype(np.uint8).tobytes())  # PGMは行0=上端

    lowres_res = resolution * f
    origin = [ox + min_x * tile_cells * resolution,
              oy + min_y * tile_cells * resolution, 0.0]
    with open(out.with_suffix('.yaml'), 'w') as fp:
        yaml.safe_dump({
            'image': pgm_path.name,
            'resolution': lowres_res,
            'origin': origin,
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
            'mode': 'trinary',
        }, fp, default_flow_style=False, sort_keys=False)

    print(f'wrote {pgm_path} ({W}x{H} @ {lowres_res} m/cell, '
          f'{W * lowres_res:.1f} x {H * lowres_res:.1f} m)')
    print(f'wrote {out.with_suffix(".yaml")}  origin={origin[:2]}')


if __name__ == '__main__':
    main()
