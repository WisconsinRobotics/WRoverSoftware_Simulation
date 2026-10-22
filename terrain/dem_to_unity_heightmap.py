"""Convert a GeoTIFF DEM (e.g. an OpenTopography TIN job) into a Unity terrain heightmap.

open topography site used: https://opentopography.org/

Outputs, written to terrain/<name>/ by default:
  <name>.raw          16-bit little-endian heightmap for Terrain Settings > Import Raw
  <name>.json         Unity import settings plus georeferencing of the terrain origin
  <name>_preview.png  north-up hillshade of exactly what was written, to check orientation

Usage:
  python terrain/dem_to_unity_heightmap.py ~/Downloads/output.tin.tar.gz --name engineering_campus

Requires: pip install rasterio numpy scipy pillow
"""

import argparse
import json
import tarfile
from pathlib import Path

import numpy as np
import rasterio
from PIL import Image, ImageDraw
from scipy import ndimage

UNITY_RESOLUTIONS = [33, 65, 129, 257, 513, 1025, 2049, 4097]
US_SURVEY_FOOT_M = 1200 / 3937


def extract_tif(archive, dest):
    """Pull the first .tif out of an OpenTopography .tar.gz download."""
    with tarfile.open(archive) as tar:
        member = next((m for m in tar.getmembers() if m.name.lower().endswith(('.tif', '.tiff'))), None)
        if member is None:
            raise SystemExit(f'No .tif found in {archive}')
        with tar.extractfile(member) as src:
            dest.write_bytes(src.read())
    return dest


def largest_valid_rect(valid):
    """Largest axis-aligned rectangle containing only valid pixels -> (top, bottom, left, right), inclusive."""
    h, w = valid.shape
    heights = np.zeros(w, dtype=int)
    best_area, best = 0, None
    for r in range(h):
        heights = np.where(valid[r], heights + 1, 0)
        stack = []  # (start column, bar height)
        for c in range(w + 1):
            cur = heights[c] if c < w else 0
            start = c
            while stack and stack[-1][1] >= cur:
                s, bar = stack.pop()
                if bar * (c - s) > best_area:
                    best_area, best = bar * (c - s), (r - bar + 1, r, s, c - 1)
                start = s
            stack.append((start, cur))
    if best is None:
        raise SystemExit('DEM has no valid pixels')
    return tuple(int(v) for v in best)


def despike(dem, threshold, max_passes=10):
    """Replace isolated pixels that differ from their 3x3 median by more than threshold metres.

    Repeated so adjacent spikes (e.g. a scan-line of bad ground returns) are caught once
    their worst neighbours are fixed. Multi-pixel features like stairwells survive.
    """
    replaced = np.zeros(dem.shape, dtype=bool)
    for _ in range(max_passes):
        med = ndimage.median_filter(dem, size=3, mode='nearest')
        spikes = np.abs(dem - med) > threshold
        if not spikes.any():
            break
        dem = np.where(spikes, med, dem)
        replaced |= spikes
    return dem, int(replaced.sum())


def hillshade(dem, res_x, res_y, azimuth=315.0, altitude=45.0):
    d_row, d_col = np.gradient(dem, res_y, res_x)
    slope = np.pi / 2 - np.arctan(np.hypot(d_row, d_col))
    aspect = np.arctan2(-d_row, d_col)
    az, alt = np.radians(azimuth), np.radians(altitude)
    shaded = np.sin(alt) * np.sin(slope) + np.cos(alt) * np.cos(slope) * np.cos(az - aspect)
    return np.clip(255 * (shaded + 1) / 2, 0, 255).astype(np.uint8)


def write_preview(dem, res_x, res_y, path):
    img = Image.fromarray(hillshade(dem, res_x, res_y)).convert('RGB')
    draw = ImageDraw.Draw(img)
    w, h = img.size
    draw.text((w // 2 - 3, 4), 'N', fill=(255, 0, 0))
    draw.ellipse((2, h - 8, 8, h - 2), fill=(255, 0, 0))
    draw.text((12, h - 14), 'Unity (0,0)  +X ->', fill=(255, 0, 0))
    img.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=Path, help='GeoTIFF DEM, or the .tar.gz OpenTopography download')
    parser.add_argument('--name', required=True, help='base name for outputs, e.g. engineering_campus')
    parser.add_argument('--out-dir', type=Path, help='default: terrain/<name>/ next to this script')
    parser.add_argument('--resolution', type=int, choices=UNITY_RESOLUTIONS,
                        help='heightmap resolution; default is the smallest that keeps every source pixel')
    parser.add_argument('--despike-threshold', type=float, default=3.0,
                        help='metres from 3x3 median to count as a spike; 0 disables (default 3.0)')
    parser.add_argument('--vertical-units', choices=['m', 'ftUS'], default='m',
                        help='units of the elevation values in the GeoTIFF (default m)')
    args = parser.parse_args()

    out_dir = args.out_dir or Path(__file__).resolve().parent / args.name
    out_dir.mkdir(parents=True, exist_ok=True)

    tif = args.input
    if tif.name.lower().endswith(('.tar.gz', '.tgz', '.tar')):
        tif = extract_tif(tif, out_dir / f'{args.name}_source.tif')

    with rasterio.open(tif) as src:
        if src.crs is None or not src.crs.is_projected or src.crs.linear_units not in ('metre', 'meter'):
            raise SystemExit(f'Need a projected CRS in metres (e.g. UTM); got {src.crs}. '
                             'Re-run the OpenTopography job with a UTM output coordinate system.')
        t = src.transform
        if t.b != 0 or t.d != 0:
            raise SystemExit('Rotated rasters are not supported')
        res_x, res_y = t.a, -t.e
        dem = src.read(1, masked=True)
        crs = src.crs.to_string()

    # Crop away NoData instead of inventing terrain to fill it.
    valid = ~np.ma.getmaskarray(dem)
    top, bottom, left, right = largest_valid_rect(valid)
    dem = dem.data[top:bottom + 1, left:right + 1].astype(np.float64)
    if args.vertical_units == 'ftUS':
        dem *= US_SURVEY_FOOT_M

    n_spikes = 0
    if args.despike_threshold > 0:
        dem, n_spikes = despike(dem, args.despike_threshold)

    rows, cols = dem.shape
    resolution = args.resolution or next((r for r in UNITY_RESOLUTIONS if r >= max(rows, cols)), UNITY_RESOLUTIONS[-1])

    # Unity samples are terrain vertices, so map them onto source pixel centres edge to edge.
    rr, cc = np.meshgrid(np.linspace(0, rows - 1, resolution), np.linspace(0, cols - 1, resolution), indexing='ij')
    heights = ndimage.map_coordinates(dem, [rr, cc], order=1, mode='nearest')

    h_min, h_max = float(dem.min()), float(dem.max())
    if h_max <= h_min:
        raise SystemExit('DEM is perfectly flat; nothing to import')

    # GeoTIFF row 0 is north; Unity's first raw row is the terrain's z = 0 edge, so write south first.
    normalized = (heights[::-1] - h_min) / (h_max - h_min)
    raw = np.round(normalized * 65535).astype('<u2')
    raw_path = out_dir / f'{args.name}.raw'
    raw.tofile(raw_path)

    size_x, size_z, size_y = (cols - 1) * res_x, (rows - 1) * res_y, h_max - h_min
    # Unity terrain local (0, 0, 0) is the centre of the south-west pixel of the crop at h_min.
    origin_e = t.c + (left + 0.5) * res_x
    origin_n = t.f - (bottom + 0.5) * res_y

    meta = {
        'source': tif.name,
        'source_crs': crs,
        'source_pixel_size_m': [res_x, res_y],
        'crop_pixels': {'top': top, 'bottom': bottom, 'left': left, 'right': right, 'width': cols, 'height': rows},
        'despike_threshold_m': args.despike_threshold,
        'despiked_pixels': n_spikes,
        'elevation_m': {'min': round(h_min, 3), 'max': round(h_max, 3)},
        'unity_import_raw': {
            'depth': 'Bit 16',
            'resolution': resolution,
            'byte_order': 'Windows',
            'flip_vertically': False,
            'terrain_size': {'x': round(size_x, 3), 'y': round(size_y, 3), 'z': round(size_z, 3)},
        },
        'terrain_origin': {
            'crs': crs,
            'easting_m': round(origin_e, 3),
            'northing_m': round(origin_n, 3),
            'elevation_m': round(h_min, 3),
            'axes': '+X = east, +Z = north, +Y = up',
        },
    }
    (out_dir / f'{args.name}.json').write_text(json.dumps(meta, indent=2) + '\n')
    write_preview(dem, res_x, res_y, out_dir / f'{args.name}_preview.png')

    print(f'Cropped to {cols} x {rows} valid pixels (dropped NoData edges); despiked {n_spikes} pixels')
    print(f'Elevation {h_min:.2f} .. {h_max:.2f} m')
    print(f'Wrote {raw_path}')
    print('\nUnity: Terrain > Terrain Settings > Texture Resolutions > Import Raw...')
    print('  Depth: Bit 16 | Byte Order: Windows | Flip Vertically: off')
    print(f'  Resolution: {resolution}')
    print(f'  Terrain Size: X {size_x:.3f}  Y {size_y:.3f}  Z {size_z:.3f}')


if __name__ == '__main__':
    main()
