# Motion Detection Upgrade: GMM Pipeline

Reverse-engineered from Tapo C510W firmware (Ingenic T31, `/bin/main`).
The current `MotionDetector` uses simple frame differencing. This document
describes a full GMM-based pipeline extracted from the camera's AMS-VDR
(Advanced Motion Sensing - Video Detection & Recognition) system, ready
to replace it.

## Current vs Proposed

| Aspect | Current (`motion_detect.py`) | Proposed (GMM pipeline) |
|--------|------------------------------|------------------------|
| Algorithm | Absolute pixel diff between 2 frames | Gaussian Mixture Model background subtraction |
| Adaptivity | None (any lighting change = motion) | Learns background over time, adapts to slow changes |
| Night handling | Same thresholds always | Separate profiles for day / night-IR / night-WL |
| Noise filtering | None | Blob area filter + dust/insect filter + trajectory confirmation |
| False positives | High (shadows, clouds, auto-exposure) | Low (GMM absorbs gradual changes) |
| Compute cost | Very low | Low-moderate (still CPU-only, no GPU needed) |

## Architecture

```
RTSP frame (JPEG)
     |
     v
Downsample to WORK_WIDTH x WORK_HEIGHT (e.g. 160x90)
     |
     v
Convert to grayscale
     |
     +---> GMM Background Model (per-pixel, K gaussians)
     |          |
     |          v
     |     Foreground Mask (binary: 0=bg, 1=fg)
     |          |
     |          v
     |     Morphological cleanup (erode + dilate)
     |          |
     |          v
     |     Connected Components (blob detection)
     |          |
     |          v
     |     Filter blobs by min area
     |          |
     |          v
     |     Trajectory tracker (confirm across N frames)
     |          |
     |          v
     |     MOTION EVENT (score + bounding boxes)
     |
     +---> Frame Differencing (fast supplement, optional)
```

## GMM Background Model

Each pixel is modeled by K gaussian distributions (typically K=3-5).
A new pixel value is compared against all K gaussians. If it matches one
(within `var_thresh` standard deviations), that gaussian is updated. If
no match, the weakest gaussian is replaced.

Gaussians are sorted by weight/sigma. The top gaussians whose cumulative
weight exceeds `background_ratio` are considered "background".

### Parameters (from Tapo firmware `ams.config`)

```python
# GMM core parameters
GMM_NUM_GAUSSIANS = 3           # K distributions per pixel
GMM_LEARNING_RATE = 0.02        # alpha: how fast background adapts (2% per frame)
GMM_BACKGROUND_RATIO = 0.7     # proportion of gaussians that form background
GMM_WEIGHT_INIT = 12            # initial weight for new gaussian
GMM_NOISE_SIGMA = 5             # base noise level (varies by sensitivity)
GMM_VAR_THRESH = 4.0            # std deviations to classify as foreground
GMM_HISTORY = 200               # frames of history for model convergence
```

### Per-pixel state

```python
@dataclass
class GaussianComponent:
    mean: float       # center of this gaussian
    variance: float   # spread
    weight: float     # mixing weight (sums to 1.0 across K)
```

Each pixel stores K of these. Total memory for 160x90 frame with K=3:
`160 * 90 * 3 * 3_floats * 4_bytes = ~622 KB` (fits easily in RAM).

### Update algorithm (per pixel, per frame)

```
for each pixel value x:
    matched = False
    for each gaussian g (sorted by weight/sigma descending):
        if |x - g.mean| < VAR_THRESH * sqrt(g.variance):
            # Match: update this gaussian
            g.weight = (1 - LEARNING_RATE) * g.weight + LEARNING_RATE
            rho = LEARNING_RATE / g.weight
            g.mean = (1 - rho) * g.mean + rho * x
            g.variance = (1 - rho) * g.variance + rho * (x - g.mean)^2
            matched = True
            break
        else:
            g.weight = (1 - LEARNING_RATE) * g.weight

    if not matched:
        # Replace weakest gaussian
        weakest = gaussian with lowest weight
        weakest.mean = x
        weakest.variance = NOISE_SIGMA^2
        weakest.weight = WEIGHT_INIT / sum_of_all_weights

    # Normalize weights
    normalize all g.weight so they sum to 1.0

    # Classify: is this pixel foreground?
    sort gaussians by weight/sqrt(variance) descending
    cumulative_weight = 0
    is_background = False
    for g in sorted_gaussians:
        cumulative_weight += g.weight
        if |x - g.mean| < VAR_THRESH * sqrt(g.variance):
            is_background = True
            break
        if cumulative_weight > BACKGROUND_RATIO:
            break

    foreground_mask[pixel] = 0 if is_background else 1
```

## Foreground Cleanup

After GMM produces a binary mask:

```python
# 1. Morphological operations to remove noise
MORPH_ERODE_SIZE = 3     # removes isolated noise pixels
MORPH_DILATE_SIZE = 5    # reconnects nearby foreground regions

# 2. Connected component analysis
MIN_BLOB_AREA_PX = 20    # minimum blob size (at 160x90 resolution)
                          # Tapo uses 32-120 depending on sensitivity

# 3. Edge area filter (Tapo: md_edge_area_thresh)
# Reject blobs that are mostly on frame edges (camera auto-exposure artifacts)
EDGE_MARGIN_PCT = 10      # ignore blobs within 10% of frame border
```

## Dust/Insect Filter

IR illumination attracts insects at night, causing bright fast-moving spots.
The Tapo firmware includes a `dust_filter` module.

```python
# Dust filter heuristic:
# - Very small blob (< DUST_MAX_AREA_PX)
# - Very high speed (appears/disappears within 1-2 frames)
# - Located near IR LEDs (usually top/center of frame)
DUST_MAX_AREA_PX = 15
DUST_MAX_LIFETIME_FRAMES = 2
DUST_IR_ZONE_TOP_PCT = 30     # top 30% of frame is IR hotspot
```

## Trajectory Confirmation

The Tapo requires `md_min_object_on_trajectory = 4` consecutive detections
before confirming motion. This eliminates transient false positives.

```python
TRAJECTORY_CONFIRM_FRAMES = 4   # blob must persist N frames to confirm
TRAJECTORY_SEARCH_RADIUS_PX = 8 # how far a blob can move between frames
TRAJECTORY_MAX_GAP_FRAMES = 3   # max frames without detection before dropping
```

### Tracker state

```python
@dataclass
class TrackedBlob:
    id: int
    centroid: tuple[float, float]
    area: float
    frames_seen: int          # consecutive frames detected
    frames_missing: int       # consecutive frames NOT detected
    confirmed: bool           # frames_seen >= TRAJECTORY_CONFIRM_FRAMES
```

Each frame: match current blobs to tracked blobs by nearest centroid
within `SEARCH_RADIUS`. Unmatched blobs start new tracks. Unmatched
tracks increment `frames_missing`; drop if exceeds `MAX_GAP`.

## Day/Night Sensitivity Profiles

The Tapo uses 30 profiles: 10 sensitivity levels x 3 lighting conditions.
For robot-brain, 3 profiles are sufficient (selected by ambient light or
camera mode):

```python
PROFILES = {
    "day": {
        "noise_sigma": 5,
        "var_thresh": 4.0,
        "min_blob_area": 45,
        "md_area_thresh_pct": 2.3,    # 2.3% of frame must change
        "edge_filter": 0.45,
        "learning_rate": 0.02,
    },
    "night_ir": {
        "noise_sigma": 7,             # more noise in IR
        "var_thresh": 5.0,            # stricter threshold
        "min_blob_area": 60,          # larger blobs only
        "md_area_thresh_pct": 2.7,
        "edge_filter": 0.35,
        "learning_rate": 0.02,
        "dust_filter": True,          # enable insect filter
    },
    "night_color": {
        "noise_sigma": 10,            # color night has most noise
        "var_thresh": 5.0,
        "min_blob_area": 60,
        "md_area_thresh_pct": 2.3,
        "edge_filter": 0.45,
        "learning_rate": 0.02,
    },
}
```

The Tapo's 10-level sensitivity maps `md_area_thresh` from 0.5% (level 10,
most sensitive) to 13.5% (level 1, least sensitive). For robot-brain, expose
a single `sensitivity` 1-10 integer in config.yaml and interpolate:

```python
# area_thresh_pct for sensitivity level (1=low, 10=high)
SENSITIVITY_AREA_MAP = [13.5, 8.5, 5.0, 3.3, 2.3, 1.55, 1.2, 0.73, 0.63, 0.50]
```

## Motion Score Calculation

Replace the current simple percentage with a weighted score:

```python
def compute_motion_score(blobs, frame_area, profile):
    """
    Score 0-100 combining:
    - fg_area_pct: percentage of frame that is foreground
    - blob_count: number of significant blobs
    - confirmed_count: blobs with trajectory confirmation
    """
    total_fg_area = sum(b.area for b in blobs)
    fg_pct = (total_fg_area / frame_area) * 100

    confirmed = [b for b in blobs if b.confirmed]
    if not confirmed:
        return 0.0  # no confirmed trajectories = no motion

    # Scale by area threshold
    area_thresh = profile["md_area_thresh_pct"]
    raw_score = min((fg_pct / area_thresh) * 50, 100)

    return raw_score
```

## Implementation Plan

### Phase 1: GMM core (replace frame differencing)

File: `perception/motion_detect.py`

1. Add `GMMBackgroundModel` class with per-pixel K-gaussian state
2. Replace `MotionDetector._compute_score()` to use GMM foreground mask
3. Add morphological cleanup (can use PIL or pure Python)
4. Keep the same `feed(jpeg_bytes) -> float` API
5. Add `warmup_frames` counter (GMM needs ~200 frames to converge;
   during warmup, fall back to current frame-diff method)

### Phase 2: Blob detection + filtering

1. Connected component labeling on foreground mask (pure Python flood-fill
   or scipy.ndimage if available)
2. Filter by `MIN_BLOB_AREA_PX`
3. Add edge-area filter
4. Return blob list with centroids + areas

### Phase 3: Trajectory tracker

1. Simple nearest-neighbor tracker across frames
2. Require `TRAJECTORY_CONFIRM_FRAMES` before reporting motion
3. Expose confirmed blob bounding boxes for VLM (crop region of interest
   instead of full frame = faster + more accurate VLM)

### Phase 4: Night mode + dust filter

1. Accept `lighting_mode` parameter in config per camera
2. Auto-detect via frame brightness histogram (mean < 50 = night)
3. Add dust filter for IR cameras

### Phase 5: Config integration

Add to `config.yaml` per camera:

```yaml
rtsp_cameras:
  - name: parking
    url: "rtsp://CParking:B8004ccd21@10.0.2.73:554/stream1"
    zone_waypoint: zona_parking
    scan_interval_s: 5
    sensitivity: 7                # 1-10 (maps to area threshold)
    lighting_mode: auto           # auto | day | night_ir | night_color
    dust_filter: true             # enable for IR cameras
    trajectory_confirm: 4         # frames to confirm motion
```

## Dependencies

All phases can be implemented in pure Python + PIL (already a dependency).
OpenCV is optional but recommended for Phase 2+ (morphological ops, connected
components). No GPU or neural network required.

## Reference: Tapo AMS-VDR Config Dump

Source: `ams.config` extracted from Tapo C510W firmware v1.0.9

```
Work resolution:     352x288 (vframe), 88x72 (sub_vframe for GMM)
Motion detection:    256x144, checked every 2 frames
People detection:    ACF classifier, 1300x900 scale, every 5 frames
GMM:                 learning_rate=0.02, background_ratio=0.7, history=200
Sensitivity levels:  10 per lighting condition (day, night_ir, night_wl)
Blob threshold:      32-120 px depending on sensitivity
Trajectory:          min 4 consecutive detections, search radius 65x48
Dust filter:         available but disabled by default (md_dust_flag=0)
```
