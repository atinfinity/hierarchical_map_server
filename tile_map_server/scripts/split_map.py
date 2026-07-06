#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""map_server形式の地図(YAML + 画像)をtile_map_server用のタイルセットに分割する。

usage:
    split_map.py --map bigmap.yaml --tile-size-cells 1000 --out map_tiles/

タイルセットの原点はタイルサイズの整数倍に切り下げて整列し、端数は未知セルで
パディングする。全面未知のタイルはファイル出力しない。
"""

import argparse
import math
import pathlib
import sys

import numpy as np
import yaml

UNKNOWN, FREE, OCCUPIED = -1, 0, 100
# map_server標準のPGM色規約
PGM_VALUE = {OCCUPIED: 0, FREE: 254, UNKNOWN: 205}


def read_pgm_p5(path: pathlib.Path) -> np.ndarray:
    """バイナリPGM(P5)を読む。Pillow非依存のフォールバック。"""
    data = path.read_bytes()
    tokens = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i] != ord('\n'):
                i += 1
            continue
        start = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    i += 1  # maxval直後の区切り1文字
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or maxval > 255:
        raise ValueError(f'unsupported PGM format: {path}')
    img = np.frombuffer(data[i:i + width * height], dtype=np.uint8)
    if img.size != width * height:
        raise ValueError(f'truncated PGM data: {path}')
    return img.reshape(height, width)


def read_image(path: pathlib.Path) -> np.ndarray:
    """画像をグレースケールのuint8配列(行0=上端)として読む。"""
    try:
        from PIL import Image
        Image.MAX_IMAGE_PIXELS = None  # 広域地図は巨大なので爆弾判定を無効化
        with Image.open(path) as im:
            return np.asarray(im.convert('L'), dtype=np.uint8)
    except ImportError:
        if path.suffix.lower() != '.pgm':
            sys.exit(f'error: Pillow is required to read {path.suffix} images '
                     '(pip install pillow), or use a P5 PGM input')
        return read_pgm_p5(path)


def write_pgm_p5(path: pathlib.Path, img: np.ndarray) -> None:
    with open(path, 'wb') as f:
        f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode())
        f.write(img.astype(np.uint8).tobytes())


def to_trinary(img: np.ndarray, negate: bool,
               occupied_thresh: float, free_thresh: float) -> np.ndarray:
    """輝度値をmap_server互換のトリナリ占有値 {-1, 0, 100} に変換する。"""
    v = img.astype(np.float64) / 255.0
    occ = v if negate else 1.0 - v
    out = np.full(img.shape, UNKNOWN, dtype=np.int8)
    out[occ > occupied_thresh] = OCCUPIED
    out[occ < free_thresh] = FREE
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, help='map_server形式のYAML')
    parser.add_argument('--tile-size-cells', type=int, default=1000,
                        help='タイル一辺のセル数 (default: 1000)')
    parser.add_argument('--out', required=True, help='出力ディレクトリ')
    args = parser.parse_args()

    map_yaml = pathlib.Path(args.map)
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)

    resolution = float(meta['resolution'])
    origin = meta['origin']
    if len(origin) >= 3 and abs(float(origin[2])) > 1e-9:
        sys.exit('error: maps with a rotated origin (yaw != 0) are not supported. '
                 'Bake the rotation into the image first.')
    negate = bool(int(meta.get('negate', 0)))
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))
    mode = meta.get('mode', 'trinary')
    if mode != 'trinary':
        sys.exit(f'error: map mode "{mode}" is not supported (trinary only)')

    image_path = pathlib.Path(meta['image'])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    print(f'reading {image_path} ...')
    img = read_image(image_path)
    occ = to_trinary(img, negate, occupied_thresh, free_thresh)
    occ = np.flipud(occ)  # 行0=下端(ワールド+y方向に行が増える向き)に揃える
    height, width = occ.shape
    print(f'map: {width}x{height} cells @ {resolution} m/cell '
          f'({width * resolution:.1f} x {height * resolution:.1f} m)')

    tile_cells = args.tile_size_cells
    tile_m = tile_cells * resolution

    # タイルセット原点をタイルサイズの整数倍に切り下げて整列
    ts_origin_x = math.floor(float(origin[0]) / tile_m) * tile_m
    ts_origin_y = math.floor(float(origin[1]) / tile_m) * tile_m
    pad_x = round((float(origin[0]) - ts_origin_x) / resolution)
    pad_y = round((float(origin[1]) - ts_origin_y) / resolution)

    n_tiles_x = math.ceil((pad_x + width) / tile_cells)
    n_tiles_y = math.ceil((pad_y + height) / tile_cells)

    canvas = np.full((n_tiles_y * tile_cells, n_tiles_x * tile_cells), UNKNOWN, dtype=np.int8)
    canvas[pad_y:pad_y + height, pad_x:pad_x + width] = occ

    out_dir = pathlib.Path(args.out)
    tiles_dir = out_dir / 'tiles'
    tiles_dir.mkdir(parents=True, exist_ok=True)

    lut = np.full(256, PGM_VALUE[UNKNOWN], dtype=np.uint8)
    lut[FREE] = PGM_VALUE[FREE]
    lut[OCCUPIED] = PGM_VALUE[OCCUPIED]

    written = 0
    for iy in range(n_tiles_y):
        for ix in range(n_tiles_x):
            tile = canvas[iy * tile_cells:(iy + 1) * tile_cells,
                          ix * tile_cells:(ix + 1) * tile_cells]
            if np.all(tile == UNKNOWN):
                continue  # 全面未知のタイルは出力しない
            pgm = lut[tile.astype(np.uint8)]
            write_pgm_p5(tiles_dir / f'tile_{ix}_{iy}.pgm', np.flipud(pgm))
            written += 1

    tileset = {
        'resolution': resolution,
        'tile_size_cells': tile_cells,
        'origin': [ts_origin_x, ts_origin_y],
        'negate': 0,
        'occupied_thresh': occupied_thresh,
        'free_thresh': free_thresh,
    }
    with open(out_dir / 'tileset.yaml', 'w') as f:
        yaml.safe_dump(tileset, f, default_flow_style=False, sort_keys=False)

    print(f'wrote {written}/{n_tiles_x * n_tiles_y} tiles '
          f'({tile_cells} cells = {tile_m:.1f} m each) to {tiles_dir}/')
    print(f'tileset origin: ({ts_origin_x}, {ts_origin_y})')
    print(f'wrote {out_dir / "tileset.yaml"}')


if __name__ == '__main__':
    main()
