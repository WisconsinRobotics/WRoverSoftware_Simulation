import numpy as np
from osgeo import gdal, osr
from PIL import Image
import os
from pathlib import Path
from scipy.ndimage import gaussian_filter, sobel
import argparse
import sys

# Enables exceptions for debugging
gdal.UseExceptions()

def normalize_path(path_input: str | Path | None) -> Path | None:
  """Resolves paths safely across Windows, WSL, and Linux native environments.

  Translates Windows drive letters (e.g. C:\\foo or C:/foo) into WSL /mnt/c/foo
  when executing under Linux.
  """
  if path_input is None:
    return None

  p_str = str(path_input).strip('\'"')

  # If running inside Linux/WSL but a Windows-style path was passed. If running in Windows then we simply return the input path, so skip the if
  if sys.platform != "win32" and len(p_str) >= 2 and p_str[1] == ":":
    drive = p_str[0].lower()
    rest = p_str[2:].replace("\\", "/").lstrip("/")
    return Path(f"/mnt/{drive}/{rest}").resolve()

  return Path(p_str).expanduser().resolve()

def extract_rgb_for_dem(sid_path : Path, dem_tif : Path, rgb_out : Path, run_debug: bool):
  """
    This function extracts the color graded map isolated with the pre-cropped DEM site.
    Args:
      sid_path: The path to sid file containing the RGB and NIR data
      dem_tif: The path to where your final DEM is stored
      rgb_out: The path where you want to store your output color graded map
      run_debug: Whether to run extra debug information

    Returns: None
  """
  if (run_debug):
    print("=== 1. CHECKING SOURCE SID ===")
    sid_ds = gdal.Open(str(sid_path))
    if not sid_ds:
      print(f"ERROR: Could not open .sid file at {sid_path}")
      sys.exit(1)
    else:
      print(f"SID Driver:     {sid_ds.GetDriver().ShortName}")
      print(f"SID Dimensions: {sid_ds.RasterXSize} x {sid_ds.RasterYSize}")
      print(f"SID Bands:      {sid_ds.RasterCount}")
      sid_srs = sid_ds.GetProjection()
      print(f"SID Projection: {'DEFINED' if sid_srs else 'EMPTY / NONE'}")

      # Calculate SID Bounding Box
      sgt = sid_ds.GetGeoTransform()
      sid_min_x = sgt[0]
      sid_max_y = sgt[3]
      sid_max_x = sid_min_x + sgt[1] * sid_ds.RasterXSize
      sid_min_y = sid_max_y + sgt[5] * sid_ds.RasterYSize
      print(f"SID X Range:    {sid_min_x:.1f} to {sid_max_x:.1f}")
      print(f"SID Y Range:    {sid_min_y:.1f} to {sid_max_y:.1f}")

  # Get metric bounds from the site DEM (autonomous site only)
  dem_ds = gdal.Open(str(dem_tif))
  if not dem_ds:
    print(f"ERROR: Could not open DEM file at {dem_tif}")
    sys.exit(1)
  # 2D affine transformation map - maps discrete image pixel indices to physical metric coordinates (Easting Northing) X,Y (Top-left) + [(deltaX, 0), (0, deltaY)].
  # In a flattened representation we have X: 0, delta X: 1, x skew: 2, Y: 3, y_skew: 4, delta Y: 5
  dem_gt = dem_ds.GetGeoTransform() 
  min_x = dem_gt[0]
  max_y = dem_gt[3]
  max_x = min_x + dem_gt[1] * dem_ds.RasterXSize # How big a row
  min_y = max_y + dem_gt[5] * dem_ds.RasterYSize # dem_gt[5] is negative
  print(f"DEM X Range:    {min_x:.1f} to {max_x:.1f}")
  print(f"DEM Y Range:    {min_y:.1f} to {max_y:.1f}")

  # Warp and Crop from the MrSid (Multi Resolution Seamless Image Database)- now we have a EPSG:32612 dataset for our autonomous region
  clipped_ds = gdal.Warp(
      "",
      sid_path,
      format="MEM",
      outputBounds=[min_x, min_y, max_x, max_y],
      srcSRS="EPSG:26912",
      dstSRS="EPSG:32612",
      resampleAlg=gdal.GRIORA_Bilinear,
  )

  if (run_debug):
    if not clipped_ds:
      print("ERROR: gdal.Warp failed completely and returned None.")
    else:
      print(f"Clipped Size:   {clipped_ds.RasterXSize} x {clipped_ds.RasterYSize}")
      band1 = clipped_ds.GetRasterBand(1).ReadAsArray()
      print(f"Band 1 Min Val: {band1.min()}")
      print(f"Band 1 Max Val: {band1.max()}")
      print(f"Band 1 Mean:    {band1.mean():.2f}")
      if band1.max() == 0:
        print("WARNING: Clipped raster contains ONLY zeros (black image)!")
      else:
        print("SUCCESS: Valid pixel data retrieved!")

  gdal.Translate(
      str(rgb_out),
      clipped_ds,
      bandList=[1,2,3],
      creationOptions=[
          "COMPRESS=DEFLATE",
          "PREDICTOR=2", # Horizontal differencing - replaces each value in a sequence with the difference between that value and immediate neighbor optim compression
          "TILED=YES",
      ],
  )

  print(f"Extracted RGB texture to path: {rgb_out}")

def get_DEM_scale(dem_tif : Path):
  ds = gdal.Open(str(dem_tif))
  if not ds:
    print(f"ERROR: Could not open DEM file at {dem_tif}")
    sys.exit(1)

  # 1. Image Resolution and Affine Transform
  cols = ds.RasterXSize
  rows = ds.RasterYSize
  gt = ds.GetGeoTransform()

  pixel_w = abs(gt[1])
  pixel_h = abs(gt[5])

  # 2. Metric Footprint (Meters in UTM)
  width_m = cols * pixel_w
  length_m = rows * pixel_h
  area_m2 = width_m * length_m
  area_km2 = area_m2 / 1_000_000.0
  area_sq_mi = area_m2 / 2_589_988.110336

  min_x = gt[0]
  max_x = min_x + (gt[1] * cols)
  max_y = gt[3]
  min_y = max_y + (gt[5] * rows)

  # 3. Transform to Geographic WGS84 (Latitude / Longitude)
  src_srs = osr.SpatialReference()
  src_srs.ImportFromWkt(ds.GetProjection())

  wgs84_srs = osr.SpatialReference()
  wgs84_srs.ImportFromEPSG(4326)
  wgs84_srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)

  tx = osr.CoordinateTransformation(src_srs, wgs84_srs)
  lon_min, lat_min, _ = tx.TransformPoint(min_x, min_y)
  lon_max, lat_max, _ = tx.TransformPoint(max_x, max_y)

  # Print Summary
  print("=" * 50)
  print(f"FILE: {dem_tif}")
  print("=" * 50)
  print(f"Grid Dimensions:      {cols} x {rows} pixels")
  print(f"Ground Sample Dist:   {pixel_w:.3f} m x {pixel_h:.3f} m per pixel")
  print(f"Terrain Width (X):    {width_m:.2f} meters ({width_m / 1609.344:.3f} miles)")
  print(f"Terrain Length (Z):   {length_m:.2f} meters ({length_m / 1609.344:.3f} miles)")
  print(f"Total Area Covered:   {area_m2:,.1f} m²")
  print(f"                      {area_km2:.3f} km²")
  print(f"                      {area_sq_mi:.3f} sq miles ({area_sq_mi * 100:.1f}% of 1 sq mi)")
  print("-" * 50)
  print("UTM EPSG:32612 Bounds:")
  print(f"  Easting  (X): [{min_x:.2f}, {max_x:.2f}]")
  print(f"  Northing (Y): [{min_y:.2f}, {max_y:.2f}]")
  print("-" * 50)
  print("WGS84 Lat/Lon Extent:")
  print(f"  Longitude (W): [{lon_min:.6f}°, {lon_max:.6f}°]")
  print(f"  Latitude  (N): [{lat_min:.6f}°, {lat_max:.6f}°]")
  print("=" * 50)

def create_unity_assets(dem_tif : Path, rgb_tif : Path, unity_env_dir : Path):
  """
    This function creates the terrain assets for use in the simulation
    Args:
    dem_tif: Path to DEM file
    rgb_tif: Path to rgb color file
    unity_env_dir: Path to your unity assets environments folder

    It saves the .raw file along with an albedo color map
  """
  unity_env_dir.mkdir(parents=True, exist_ok=True)
  raw_out = unity_env_dir / "mdrs_autonomous_heightmap_2049. raw"
  png_out = unity_env_dir / "mdrs_autonomous_albedo_4096.png"

  # Process Heightmap for Unity. A unity object requires 2^n - 1 in res
  # Some calculations:
  # Dataset is a square mile for Autonomous: We have a roughly 1600 X 1600 m site.
  # At resolution of 2049x2049 we get: 1600/2049 which is a 0.78 resolution of true values and the rest as interpolation. Keeps memory low
  # Remains performant
  #

  dem_ds = gdal.Open(str(dem_tif))
  resampled_mem = gdal.Warp(
    "",
    dem_ds,
    format="MEM",
    width=2049,
    height=2049,
    resampleAlg=gdal.GRIORA_Bilinear,
  )

  elev_array = resampled_mem.GetRasterBand(1).ReadAsArray().astype(np.float32)

  # If there are any NULL pixels replace them with the lowest possible pixel value
  nodata = resampled_mem.GetRasterBand(1).GetNoDataValue()
  if nodata is not None:
    elev_array[elev_array == nodata] = np.nanmin(elev_array[elev_array != nodata])


  min_elev = float(np.min(elev_array))
  max_elev = float(np.max(elev_array))
  delta_elev = max_elev - min_elev

  normalized_elev = (elev_array - min_elev) / delta_elev
  raw_data = (normalized_elev * float(2**16 - 1)).astype(np.uint16) # Scale to 16 bit unsigned

  raw_data = np.flipud(raw_data) # Flip order from "top to bottom" to "bottom to top"

  # <u2 is saying to store it in little endian, unsigned with each data point occupying 2 bytes in memory
  raw_data.astype("<u2").tofile(raw_out)

  print(f"=== UNITY TERRAIN PARAMETERS ===")
  print(f"File Saved:     {raw_out}")
  print(f"Terrain Width (X):  1568") # Verified by running get_DEM_scale if one wants
  print(f"Terrain Length (Z): 1558")
  print(f"Terrain Height (Y): {delta_elev:.3f}")
  print(f"Base Elevation Y:   {min_elev:.3f}")

  gdal.Translate(
    str(png_out),
    str(rgb_tif),
    format="PNG",
    width=4096,
    height=4096,
  )
  print(f"Saved Albedo Map: {png_out}")

def generate_normal_map(dem_tif : Path, rgb_tif : Path, unity_env_dir : Path, size : int = 4096, micro_strength : float = 1.5):
  """
  Args:
  size - how much resampling we want to do
  micro_strength - How much weighting to give our NAIP data. At 1.5 a 10% brightness crease creates about a 15 cm elev change. Tunable through Unity as part of surface normal map
  """
  unity_env_dir.mkdir(parents=True, exist_ok=True)
  out_normal_png = unity_env_dir / "mdrs_autonomous_normal_map.png"

  dem_ds = gdal.Warp(
    "", 
    str(dem_tif), 
    format="MEM", 
    width=size, 
    height=size, 
    resampleAlg=gdal.GRIORA_Cubic,
    )

  rgb_ds = gdal.Warp(
    "",
    str(rgb_tif),
    format="MEM",
    width=size,
    height=size,
    resampleAlg=gdal.GRIORA_Cubic,
  )

  elev = dem_ds.GetRasterBand(1).ReadAsArray().astype(np.float32)
  r = rgb_ds.GetRasterBand(1).ReadAsArray().astype(np.float32) / 255.0
  g = rgb_ds.GetRasterBand(2).ReadAsArray().astype(np.float32) / 255.0
  b = rgb_ds.GetRasterBand(3).ReadAsArray().astype(np.float32) / 255.0

  lum = 0.299 * r + 0.587 * g + 0.114 * b # Done using Rec 601 standard for luminosity

  # 1. Band Pass Filtering (Difference of Gaussians) - only allow a range of frequencies to pass through
  # sigma=1.0 strips high-frequency sensor/compression noise (Canny Step 1). If too grainy, raise this
  # sigma=4.0 strips macro lighting/albedo color shifts, if banks still feel soft raise further. If variation unnatural - try lowering
  band_pass = gaussian_filter(lum, sigma=1.0) - gaussian_filter(lum, sigma=5.0)

  # Combine DEM with NAIP features
  hybrid_height = elev + (band_pass * micro_strength)

  # 2. Compute 3x3 Sobel Directional Derivatives. High pass filtering for building gradient map. [Note: not the same as edge/boundary detection]
  scale_factor = 8 * 0.383 # Sobel kernel sums weight 8 across 2 pixel span. We have 0.383 meters per pixel. so dx = 0.383 * 8
  dz_dx = sobel(hybrid_height, axis=1) / scale_factor # Emphasize vertical edges
  dz_dy = sobel(hybrid_height, axis=0) / scale_factor # Emphasize horizontal edges

  # 3. Construct Surface vectors - using surface gradient rule from multivar calc (-dz / dx, -dy / dx)
  nx = -dz_dx
  ny = dz_dy # Double negative here. Unity North +, South -, so north component is - dy /dx. But in numpy north at top south at bottom, so index increases going south
  nz = np.ones_like(elev)

  norm = np.sqrt(nx**2 + ny**2 + nz**2)
  nx, ny, nz = nx / norm, ny / norm, nz / norm

  # Encode the the vectors (nx, ny, nz) in a 2D RGB image file.
  # Each nx, ny, nz ranges from [-1.0, 1.0] in East/West, North/South and Vertical/Outward. PNG's cannot store negative numbers, so shift and compress
  normal_rgb = np.stack(
    [(nx + 1.0) / 2, (ny + 1.0) / 2, (nz + 1.0) / 2], axis=-1
  )

  normal_uint8 = (normal_rgb * 255.0).astype(np.uint8)
  Image.fromarray(normal_uint8, mode="RGB").save(str(out_normal_png))
  print(f"Exported noise-filtered 4K normal map: {out_normal_png}")


def parse_arguments():
  parser = argparse.ArgumentParser(
    description=(
      "MDRS Terrain Pre-Processing Pipeline for Unity Sim (Cross-Platform WSL/Linux/Windows)"
    ),
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  )

  # Paths control
  parser.add_argument(
    "--data-dir",
    type=str,
    default=None,
    help=(
      "Base Directory containing Raw MDRS rasters (The MDRS Terrain Files) is default and should ideally point to that as well"
    ),
  )

  parser.add_argument(
    "--dem-tif",
    type=str,
    default=None,
    help=(
      "Explicit path to mdrs_32612_dem_final.tif"
    ),
  )

  parser.add_argument(
    "--sid-path",
    type=str,
    default=None,
    help=(
      "Explicit path to NAIP2024_Wayne.sid"
    ),
  )

  parser.add_argument(
    "--rgb-tif",
    type=str,
    default=None,
    help=(
      "Explicit path to mdrs_rgb_clipped.tif"
    )
  )

  parser.add_argument(
    "--unity-env-dir",
    type=str,
    default=None,
    help=(
      "Destination directory for Unity assets (e.g."
          " Assets/Environments/MDRS)"
    )
  )

  parser.add_argument(
    "--micro-strength",
    type=float,
    default=1.5,
    help=(
      "Weight of NAIP band pass micro relief (tunable via Unity interface normals)"
    )
  )

  parser.add_argument(
    "--size",
    type=int,
    default=4096,
    help=(
      "Target square resolution for normal and albedo maps"
    )
  )

  parser.add_argument(
    "--debug",
    action="store_true",
    help="Print extended GDAL and raster debug statistics"
  )

  # Pipeline execution control
  parser.add_argument(
    "--all",
    action="store_true",
    help="Run the end-to-end pipeline (crop, scale, assets, normal)"
  )

  parser.add_argument(
    "--extract_rgb",
    action="store_true",
    help="Run RGB extraction from .sid file"
  )

  parser.add_argument(
    "--dem-scale",
    action="store_true",
    help="Print geographic bounds and scale summary of DEM"
  )

  parser.add_argument(
    "--create-assets",
    action="store_true",
    help="Create .raw heightmap and .png albedo map",
  )

  parser.add_argument(
    "--normal-map",
    action="store_true",
    help="Only regenerate the normal map (default behavior)",
  )

  return parser.parse_args()

if __name__ == '__main__':
  args = parse_arguments()
  script_dir = Path(__file__).resolve().parent

  if args.data_dir:
    data_dir = normalize_path(args.data_dir)
  else:
    sibling_data = script_dir.parent / "MDRS Terrain Files"
    data_dir = (
      sibling_data.resolve()
      if sibling_data.exists()
      else (script_dir / "data").resolve() # Extreme fallback please don't rely on this
    )

  # 2. Resolve Specific Files
  sid_path = (
      normalize_path(args.sid_path)
      if args.sid_path
      else data_dir / "NAIP2024_Wayne" / "NAIP2024_Wayne.sid"
  )
  dem_tif = (
      normalize_path(args.dem_tif)
      if args.dem_tif
      else data_dir / "Final DEM" / "mdrs_32612_dem_final.tif"
  )
  rgb_out = (
      normalize_path(args.rgb_tif)
      if args.rgb_tif
      else data_dir / "Color and Object Masks" / "mdrs_rgb_clipped.tif"
  )

  # 3. Resolve Unity Environment Output Directory
  if args.unity_env_dir:
    unity_dir = normalize_path(args.unity_env_dir)
  else:
    unity_dir = (script_dir / "Assets" / "Environments" / "MDRS").resolve()

  # 4. Pipeline
  run_everything = args.all
  run_any_specific = (
    args.extract_rgb
    or args.dem_scale
    or args.create_assets
    or args.normal_only
  )

  if not run_any_specific and not run_everything:
    generate_normal_map(
        dem_tif=dem_tif,
        rgb_tif=rgb_out,
        unity_env_dir=unity_dir,
        size=args.size,
        micro_strength=args.micro_strength,
    )
  else:
    if run_everything or args.extract_rgb:
      extract_rgb_for_dem(
          sid_path=sid_path,
          dem_tif=dem_tif,
          rgb_out=rgb_out,
          run_debug=args.debug,
      )

    if run_everything or args.dem_scale:
      get_DEM_scale(dem_tif=dem_tif)

    if run_everything or args.create_assets:
      create_unity_assets(
          dem_tif=dem_tif, rgb_tif=rgb_out, unity_env_dir=unity_dir
      )

    if run_everything or args.normal_only:
      generate_normal_map(
          dem_tif=dem_tif,
          rgb_tif=rgb_out,
          unity_env_dir=unity_dir,
          size=args.size,
          micro_strength=args.micro_strength,
      )