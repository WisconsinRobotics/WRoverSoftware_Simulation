# MDRS Terrain Processing Pipeline (`get_colored_terrain.py`)

CLI utility for processing USGS DEMs and NAIP orthophotos into Unity-compatible terrain assets (16-bit `.raw` heightmaps, 8-bit `.png` albedos, and 4K tangent-space normal maps).

---

## 1. Prerequisites & Environment

Do **not** install GDAL on native Windows. Run the script inside the Linux Docker container where GDAL C++ binaries and Python bindings are pre-installed via APT.

Ensure your container is running:
```bash
make shell-sim
# or
make sim
# or
docker compose run --rm -it main bash
```

The script features an internal path resolver (`normalize_path`). If you provide standard Windows file paths (e.g., `C:\Users\...` or `C:/Users/...`), the script automatically translates them to `/mnt/c/Users/...` inside WSL/Docker.

---

## 2. Standard Workflow

### Step A: Source Data Prep (QGIS)
1. Verify the project DEM is projected in **UTM Zone 12N (`EPSG:32612`)**.
2. Ensure the raw NAIP orthophoto (`.sid`) covers the autonomous run site coordinates.
3. Keep the raster files in `MDRS Terrain Files/` in the repository under Unity's section (that is the part which you will work with actively on Windows).

### Step B: Generate Assets (Inside Container)
Run the script from inside the container, pointing `--unity-env-dir` directly to your local Windows Unity project:

Example path (for me):
```bash
python3 get_colored_terrain.py \
  --unity-env-dir "C:\Users\<YOUR_USER>\workspace\WRoverSoftware_Simulation\Unity\WRover_27_Sim\Assets\Environments\MDRS"
```

### Step C: Unity Import
1. Switch to the Unity Editor window on Windows.
2. Unity will automatically detect the updated textures in `Assets/Environments/MDRS/`.
3. In your `MDRS_Surface_Layer` asset:
   * Assign `mdrs_autonomous_albedo_4096.png` to **Albedo**.
   * Assign `mdrs_autonomous_normal_map.png` to **Normal Map** (set **Normal Scale** between `1.0` and `1.5`).
   * Set **Wrap Mode** to `Repeat` and tile size to `X: 1568`, `Y: 1558`.

---

## 3. CLI Argument Reference

| Argument | Default | Description |
| :--- | :--- | :--- |
| `--unity-env-dir` | `<SCRIPT_DIR>/Assets/Environments/MDRS` | Target directory where generated `.raw` and `.png` assets are written. Accepts Windows or Linux paths. |
| `--data-dir` | `<REPO_ROOT>/MDRS Terrain Files` | Base folder containing raw input rasters (`Final DEM`, `NAIP2024_Wayne`, etc.). |
| `--dem-tif` | Auto-resolved from `--data-dir` | Direct path to the processed elevation GeoTIFF (`mdrs_32612_dem_final.tif`). |
| `--sid-path` | Auto-resolved from `--data-dir` | Direct path to the high-resolution NAIP MrSID file (`NAIP2024_Wayne.sid`). |
| `--rgb-tif` | Auto-resolved from `--data-dir` | Destination path for the clipped intermediate RGB raster (`mdrs_rgb_clipped.tif`). |
| `--size` | `4096` | Output square resolution (width and height) for normal and albedo textures. |
| `--micro-strength` | `1.5` | Weight of the Difference of Gaussians band-pass filter extracted from NAIP imagery into the DEM normal calculations. |
| `--all` | `False` | Executes all subroutines sequentially: RGB extraction, DEM scaling report, `.raw` heightmap generation, and normal baking. |
| `--extract-rgb` | `False` | Runs only the spatial clipping and reprojection of the `.sid` into the intermediate RGB GeoTIFF. |
| `--create-assets` | `False` | Generates the 16-bit little-endian `2049x2049` `.raw` heightmap and 4K albedo map. |
| `--normal-only` | `False` | Re-computes and bakes only the 4K normal map (default behavior if no mode flag is passed). |
| `--dem-scale` | `False` | Prints metric bounds, ground sample distance (GSD), and WGS84 coordinates to stdout. |
| `--debug` | `False` | Prints verbose GDAL metadata, band statistics, and projection validation. |

---

## 4. Common Execution Recipes

* **Regenerate Normal Map with higher surface roughness:**
  ```bash
  python3 get_colored_terrain.py --micro-strength 2.0 --unity-env-dir "C:\Users\<YOUR_USER>\...\Assets\Environments\MDRS"
  ```

* **Full End-to-End Build from Raw Data:**
  ```bash
  python3 get_colored_terrain.py --all --unity-env-dir "C:\Users\<YOUR_USER>\...\Assets\Environments\MDRS"
  ```

* **Inspect DEM Extents and Scaling Parameters:**
  ```bash
  python3 get_colored_terrain.py --dem-scale
  ```

---

## 5. Future Work: High-Resolution Multi-Biome Terrain

1. **4-Channel Biome Splatmap Generation:**
   * Extend the script to classify Morrison Formation soil biomes (Pale Bentonite Clay, Red Mudstone, Sand Washes, and Sandstone Scree) using NAIP band ratios and DEM slope gradients.
   * Export an RGBA splatmap for ingestion by `ApplyMDRSSplatmap.cs`.

2. **Micro PBR Terrain Layering:**
   * Transition from a single stretched 4K albedo to layered $2\text{ m} \times 2\text{ m}$ tiling PBR materials driven by the splatmap alphamap channels.

3. **Runtime Terramechanics Hook:**
   * Connect `SwerveDriveController.cs` raycasts to the active alphamap layer weights at the wheel contact patch to dynamically modulate rolling resistance ($C_{rr}$) and wheel slip coefficients based on local soil substrate.