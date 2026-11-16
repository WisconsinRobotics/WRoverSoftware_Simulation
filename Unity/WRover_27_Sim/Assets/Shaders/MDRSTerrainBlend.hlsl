#ifndef MDRS_TERRAIN_BLEND_INCLUDED
#define MDRS_TERRAIN_BLEND_INCLUDED

// Unpack RGB Normal Maps and reconstruct
float3 UnpackMicroNormal(float4 packedNormal, float scale){
    float3 n;
    n.xy = (packedNormal.a < 0.999) ? (packedNormal.wy * 2.0 - 1.0) : (packedNormal.xy * 2.0 - 1.0); // Unity can move the red channel into channel for higher depth sometimes
    n.xy *= scale;
    n.z = sqrt(saturate(1.0 - dot(n.xy, n.xy))); // Saturate clamps within 0 and 1. Pythagoras theorem nx^2 + ny^2 + nz^2 = 1
    return normalize(n);
}

// Reorient Normal Mapping: Householder reflection to rotate micro normal onto macro normal
float3 BlendNormalsRNM(float3 nMacro, float3 nMicro){
    float3 t = nMacro + float3(0.0, 0.0, 1.0); // Compute the bisector of the Macro normal
    float3 u = nMicro * float3(-1.0, -1.0, 1.0); // Reflect x, y about z axis

    return normalize((t * dot(t, u) / t.z) - u);
}

void EvaluateMDRSTerrain_float(
    float3 WorldPos,
    float3 CameraWorldPos,
    float3 TerrainOrigin, // Default: (-784.0, -70.8401, -779.0)
    float3 TerrainSize, // Default: (1568.0, 70.8401, 1558.0)
    UnityTexture2D MacroAlbedoTex, // Albedo map from NAIP
    UnityTexture2D MacroNormalTex, // Normal map from NAIP
    UnityTexture2D BiomeSplatTex, // Splatmap from NAIP
    // The PBR from PolyHaven (probably) or some other site
    UnityTexture2D ClayAlbedoH,  UnityTexture2D ClayNormal,  float ClayTileSize,
    UnityTexture2D MudAlbedoH,   UnityTexture2D MudNormal,   float MudTileSize,
    UnityTexture2D SandAlbedoH,  UnityTexture2D SandNormal,  float SandTileSize,
    UnityTexture2D RockAlbedoH,  UnityTexture2D RockNormal,  float RockTileSize,

    UnitySamplerState LinearClampSampler,
    UnitySamplerState LinearRepeatSampler,

    // Tunable parameters
    float HeightTransition, // delta: (Default: 0.15) - controls the height floor for determining which surface to weight on
    float MicroDetailStrength, // k : (Default: 0.85) - How much weighting to give micro stuff
    float MicroNormalStrength, // (Default: 1.20)
    float MacroNormalStrength, // (Default: 1.00)

    out float3 OutAlbedo,
    out float3 OutNormalTS,
    out float OutSmoothness,
    out float OutOcclusion
) 
{
    // Translate 3D world coords to Normalized Terrain Coordinate
    // Basically cuz terrain is offset in Unity from 0,0,0 we translate them

    float3 normCoords = saturate((WorldPos - TerrainOrigin) / max(TerrainSize, float3(1.0, 0.001, 1.0))); // Div by 0 guard
    float2 macroUV = normCoords.xz;
    float normElev = normCoords.y; // 0.0 at basin floor, and 1.0 at peak ceiling

    // Sample Ground Truth Utah Data (NAIP color, DEM normals, Splatmap weightings)
    float3 macroColor = SAMPLE_TEXTURE2D(MacroAlbedoTex, LinearClampSampler, macroUV).rgb;
    float4 macroNormalSampled = SAMPLE_TEXTURE2D(MacroNormalTex, LinearClampSampler, macroUV);
    float4 splat_weights = SAMPLE_TEXTURE2D(BiomeSplatTex, LinearClampSampler, macroUV);

    float wSum = dot(splat_weights, float4(1.0, 1.0, 1.0, 1.0)); // Sum up all the splat weights - exploit dot product
    splat_weights = (wSum > 1e-4) ? (splat_weights/wSum) : float4(1.0, 0.0, 0.0, 0.0);

    // Distance based Level of Detail (LOD) fade
    float camDist = distance(WorldPos, CameraWorldPos);
    float detailFade = 1.0 - smoothstep(30.0, 500.0, camDist); // between 0.0 and 1.0, subtract from 1 to fade more as closer to 140, beyond that just no micro detail

    // Rover eye micro tiling UV's
    float2 uvClay = WorldPos.xz / max(ClayTileSize, 0.1);
    float2 uvMud = WorldPos.xz / max(MudTileSize, 0.1);
    float2 uvSand = WorldPos.xz / max(SandTileSize, 0.1);
    float reliefHeight = WorldPos.y - TerrainOrigin.y; // Note: For steep Rock cliffs, adding (WorldPos.y - TerrainOrigin.y) prevents vertical UV stretching
    float2 uvRock = (WorldPos.xz + float2(reliefHeight * 0.35, reliefHeight * 0.35))/ max(RockTileSize, 0.1);

    // Sample Micro Albedo at MIP 0 (High res) and MIP 5 (Gaussian Kernel with 2^5 low pass)
    float4 cClay0 = SAMPLE_TEXTURE2D(ClayAlbedoH, LinearRepeatSampler, uvClay);
    float4 cMud0  = SAMPLE_TEXTURE2D(MudAlbedoH,  LinearRepeatSampler, uvMud);
    float4 cSand0 = SAMPLE_TEXTURE2D(SandAlbedoH, LinearRepeatSampler, uvSand);
    float4 cRock0 = SAMPLE_TEXTURE2D(RockAlbedoH, LinearRepeatSampler, uvRock);

    float3 cClayLow = SAMPLE_TEXTURE2D_LOD(ClayAlbedoH, LinearRepeatSampler, uvClay, 5.0).rgb;
    float3 cMudLow  = SAMPLE_TEXTURE2D_LOD(MudAlbedoH,  LinearRepeatSampler, uvMud,  5.0).rgb;
    float3 cSandLow = SAMPLE_TEXTURE2D_LOD(SandAlbedoH, LinearRepeatSampler, uvSand, 5.0).rgb;
    float3 cRockLow = SAMPLE_TEXTURE2D_LOD(RockAlbedoH, LinearRepeatSampler, uvRock, 5.0).rgb;

    // Micro Height Calcs H_i(u,v). Sample from alpha or rgb depending on packed vs unpacked
    const float3 lumaW = float3(0.299, 0.587, 0.114); // Rec 601
    float hClay = (cClay0.a < 0.99) ? cClay0.a : dot(cClay0.rgb, lumaW);
    float hMud  = (cMud0.a  < 0.99) ? cMud0.a  : dot(cMud0.rgb,  lumaW);
    float hSand = (cSand0.a < 0.99) ? cSand0.a : dot(cSand0.rgb, lumaW);
    float hRock = (cRock0.a < 0.99) ? cRock0.a : dot(cRock0.rgb, lumaW);

    // Additive Splat Blending with delta Transition window
    float4 H = float4(hClay, hMud, hSand, hRock);
    float4 h = (splat_weights > 0.001) ? (splat_weights + H) : float4(-10.0, -10.0, -10.0, -10.0); // Ignore if no weight at current Pos
    float hTop = max(max(h.x, h.y), max(h.z, h.w));
    float hfloor = hTop - max(HeightTransition, 0.01);
    float4 b = max(h - hfloor, 0.0);
    float bSum = dot(b, float4(1.0, 1.0, 1.0, 1.0));
    float4 W = (bSum > 1e-5) ? (b / bSum) : splat_weights;

    // Spatial Frequency Decomposition deltaL_i = MIP(0) - MIP(5)
    float dL_Clay = dot(cClay0.rgb - cClayLow, lumaW);
    float dL_Mud = dot(cMud0.rgb - cMudLow, lumaW);
    float dL_Sand = dot(cSand0.rgb - cSandLow, lumaW);
    float dL_Rock = dot(cRock0.rgb - cRockLow, lumaW);

    float deltaL = dot(W, float4(dL_Clay, dL_Mud, dL_Sand, dL_Rock));

    // Modulate NAIP ground truth
    float k_eff = MicroDetailStrength * detailFade;   

    // Reoriented Normals
    float3 nClay = UnpackMicroNormal(SAMPLE_TEXTURE2D(ClayNormal, LinearRepeatSampler, uvClay), MicroNormalStrength * detailFade);
    float3 nMud = UnpackMicroNormal(SAMPLE_TEXTURE2D(MudNormal, LinearRepeatSampler, uvMud), MicroNormalStrength * detailFade);
    float3 nSand = UnpackMicroNormal(SAMPLE_TEXTURE2D(SandNormal, LinearRepeatSampler, uvSand), MicroNormalStrength * detailFade);
    float3 nRock = UnpackMicroNormal(SAMPLE_TEXTURE2D(RockNormal, LinearRepeatSampler, uvRock), MicroNormalStrength * detailFade);

    float3 microNormal = normalize(W.x * nClay + W.y * nMud + W.z * nSand + W.w * nRock);
    float3 macroNormal = UnpackMicroNormal(macroNormalSampled, MacroNormalStrength);
    
   
    // Outputs
    OutAlbedo = saturate(macroColor * (1.0 + k_eff * deltaL));
    OutNormalTS = BlendNormalsRNM(macroNormal, microNormal);
    
    // Biome Smoothness and Crack Cavity Ambient Occlusion
    OutSmoothness = saturate(dot(W, float4(0.12, 0.18, 0.08, 0.25)) + 0.15 * deltaL);
    OutOcclusion = lerp(1.0, saturate(1.0 + 1.5 * min(deltaL, 0.0)), detailFade * 0.65);
    
    
    
    
    
}

#endif