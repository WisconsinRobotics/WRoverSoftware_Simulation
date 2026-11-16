import os
import io
import requests
import numpy as np
from PIL import Image

# Change to "4k" if you want full 4096x4096 (approx ~120MB download total), or "2k" for fast test
RESOLUTION = "4k"
OUTPUT_DIR = "./"
os.makedirs(OUTPUT_DIR, exist_ok=True)

POLYHAVEN_BIOMES = {
    "Clay": "mud_cracked_dry_03",
    "Mud":  "red_laterite_soil_stones",
    "Sand": "sandy_gravel_02",
    "Rock": "sandstone_cracks"
}

HEADERS = {"User-Agent": "WRover-MDRS-DigitalTwin/1.0"}

def get_map_url(files_json, candidate_keys, res):
    """Finds the matching map key (case-insensitive) and prefers png -> jpg."""
    lower_map = {k.lower(): k for k in files_json.keys()}
    for key in candidate_keys:
        if key.lower() in lower_map:
            actual_key = lower_map[key.lower()]
            res_dict = files_json[actual_key].get(res) or next(iter(files_json[actual_key].values()))
            for fmt in ["png", "jpg"]:
                if fmt in res_dict:
                    return res_dict[fmt]["url"]
    raise KeyError(f"Could not find any of {candidate_keys} in API keys: {list(files_json.keys())}")

def fetch_image(url):
    resp = requests.get(url, headers=HEADERS, timeout=30)
    resp.raise_for_status()
    return Image.open(io.BytesIO(resp.content))

for biome_prefix, asset_id in POLYHAVEN_BIOMES.items():
    print(f"[{biome_prefix}] Querying PolyHaven API for '{asset_id}' ({RESOLUTION})...")
    api_url = f"https://api.polyhaven.com/files/{asset_id}"
    resp = requests.get(api_url, headers=HEADERS, timeout=30)
    resp.raise_for_status()
    data = resp.json()    

    # 1. Fetch Diffuse (Albedo), Displacement (Height), and Normal (GL) URLs
    diff_url = get_map_url(data, ["Diffuse", "diffuse", "col", "albedo"], RESOLUTION)
    disp_url = get_map_url(data, ["Displacement", "disp", "height"], RESOLUTION)
    norm_url = get_map_url(data, ["nor_gl", "Normal", "nor_dx"], RESOLUTION)

    print(f"  -> Downloading Albedo & Displacement...")
    diff_img = fetch_image(diff_url).convert("RGB")
    disp_img = fetch_image(disp_url).convert("L")

    if disp_img.size != diff_img.size:
        disp_img = disp_img.resize(diff_img.size, Image.Resampling.BILINEAR)

    # 2. Normalize Height to full [0, 255] dynamic range and pack into Alpha channel
    disp_arr = np.array(disp_img, dtype=np.float32)
    d_min, d_max = disp_arr.min(), disp_arr.max()
    if d_max > d_min:
        disp_arr = ((disp_arr - d_min) / (d_max - d_min)) * 255.0
    else:
        disp_arr.fill(128.0)

    rgba_arr = np.dstack((np.array(diff_img, dtype=np.uint8), disp_arr.astype(np.uint8)))
    albedo_h_path = os.path.join(OUTPUT_DIR, f"{biome_prefix}_AlbedoH.png")
    Image.fromarray(rgba_arr, mode="RGBA").save(albedo_h_path)
    print(f"  -> Saved packed RGBA: {albedo_h_path}")

    # 3. Download and save OpenGL Normal Map
    print(f"  -> Downloading Normal Map...")
    norm_img = fetch_image(norm_url).convert("RGB")
    norm_path = os.path.join(OUTPUT_DIR, f"{biome_prefix}_Normal.png")
    norm_img.save(norm_path)
    print(f"  -> Saved Normal: {norm_path}")

print("\nDone!")