"""Motion detection — GMM background subtraction pipeline (B04).

Replaces simple frame differencing with a Gaussian Mixture Model (GMM)
background subtractor. Each pixel is modeled by K gaussian distributions
that learn the background over time, adapting to gradual lighting changes.

Pipeline: JPEG → downsample → grayscale → GMM → foreground mask →
          morphology → blob detection → trajectory confirmation → score.

Based on Tapo C510W AMS-VDR pipeline (see docs/motion-detection-upgrade.md).

Usage:
    detector = MotionDetector(sensitivity=7)
    score = detector.feed(frame_bytes)  # JPEG bytes
    if score > 0:
        # call VLM on confirmed motion
"""

import math
import time
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Constants — work resolution
# ---------------------------------------------------------------------------

## Downsample width for processing (matches Tapo sub_vframe scale).
WORK_WIDTH = 160
## Downsample height computed from aspect ratio at runtime.
WORK_HEIGHT_DEFAULT = 90

# ---------------------------------------------------------------------------
# GMM parameters (from Tapo ams.config)
# ---------------------------------------------------------------------------

## Number of gaussian components per pixel.
GMM_NUM_GAUSSIANS = 3
## Learning rate: how fast background adapts (2% per frame).
GMM_LEARNING_RATE = 0.02
## Proportion of gaussians forming background model.
GMM_BACKGROUND_RATIO = 0.7
## Initial weight for new gaussian component.
GMM_WEIGHT_INIT = 0.05
## Base noise sigma (varies by profile).
GMM_NOISE_SIGMA = 5.0
## Standard deviations threshold for foreground classification.
GMM_VAR_THRESH = 4.0
## Frames of history for model convergence.
GMM_WARMUP_FRAMES = 200

# ---------------------------------------------------------------------------
# Blob detection / filtering
# ---------------------------------------------------------------------------

## Minimum blob area in pixels at work resolution.
MIN_BLOB_AREA_PX = 20
## Morphological erode kernel size.
MORPH_ERODE_SIZE = 3
## Morphological dilate kernel size.
MORPH_DILATE_SIZE = 5
## Edge margin percentage (reject blobs mostly on borders).
EDGE_MARGIN_PCT = 10

# ---------------------------------------------------------------------------
# Dust / insect filter (for IR cameras)
# ---------------------------------------------------------------------------

## Maximum blob area to be classified as dust.
DUST_MAX_AREA_PX = 15
## Maximum lifetime in frames for dust classification.
DUST_MAX_LIFETIME_FRAMES = 2
## Top percentage of frame considered IR hotspot zone.
DUST_IR_ZONE_TOP_PCT = 30

# ---------------------------------------------------------------------------
# Trajectory confirmation
# ---------------------------------------------------------------------------

## Consecutive frames a blob must persist to confirm motion.
TRAJECTORY_CONFIRM_FRAMES = 4
## Search radius for matching blobs between frames (pixels).
TRAJECTORY_SEARCH_RADIUS_PX = 8
## Maximum frames without detection before dropping a track.
TRAJECTORY_MAX_GAP_FRAMES = 3

# ---------------------------------------------------------------------------
# Sensitivity → area threshold mapping (Tapo 10-level, level 1=low, 10=high)
# ---------------------------------------------------------------------------

SENSITIVITY_AREA_MAP = [13.5, 8.5, 5.0, 3.3, 2.3, 1.55, 1.2, 0.73, 0.63, 0.50]

# ---------------------------------------------------------------------------
# Day/Night profiles
# ---------------------------------------------------------------------------

PROFILES = {
    "day": {
        "noise_sigma": 5.0,
        "var_thresh": 4.0,
        "min_blob_area": 45,
        "learning_rate": 0.02,
        "dust_filter": False,
    },
    "night_ir": {
        "noise_sigma": 7.0,
        "var_thresh": 5.0,
        "min_blob_area": 60,
        "learning_rate": 0.02,
        "dust_filter": True,
    },
    "night_color": {
        "noise_sigma": 10.0,
        "var_thresh": 5.0,
        "min_blob_area": 60,
        "learning_rate": 0.02,
        "dust_filter": False,
    },
}

# Legacy constants (kept for backward compatibility in tests)
MOTION_THRESHOLD_PCT = 15
MOTION_PIXEL_DIFF_THRESHOLD = 30
MOTION_BLUR_RADIUS = 2
MOTION_DOWNSAMPLE_WIDTH = WORK_WIDTH


# ---------------------------------------------------------------------------
# Gaussian component (per-pixel)
# ---------------------------------------------------------------------------

@dataclass
class GaussianComponent:
    mean: float = 0.0
    variance: float = 25.0  # GMM_NOISE_SIGMA^2
    weight: float = 0.0


# ---------------------------------------------------------------------------
# Tracked blob
# ---------------------------------------------------------------------------

@dataclass
class TrackedBlob:
    id: int = 0
    cx: float = 0.0
    cy: float = 0.0
    area: int = 0
    frames_seen: int = 0
    frames_missing: int = 0

    @property
    def confirmed(self) -> bool:
        return self.frames_seen >= TRAJECTORY_CONFIRM_FRAMES


# ---------------------------------------------------------------------------
# MotionDetector (GMM-based)
# ---------------------------------------------------------------------------

class MotionDetector:
    """GMM-based motion detector with trajectory confirmation.

    Drop-in replacement: same `feed(jpeg_bytes) -> float` API.
    """

    def __init__(
        self,
        sensitivity: int = 7,
        lighting_mode: str = "auto",
        dust_filter: bool = False,
        trajectory_confirm: int = TRAJECTORY_CONFIRM_FRAMES,
        # Legacy compat params (ignored by GMM but accepted for API compat)
        threshold_pct: int = MOTION_THRESHOLD_PCT,
        pixel_diff: int = MOTION_PIXEL_DIFF_THRESHOLD,
    ):
        self.sensitivity = max(1, min(10, sensitivity))
        self.lighting_mode = lighting_mode
        self.dust_filter_enabled = dust_filter
        self.trajectory_confirm = trajectory_confirm
        self.threshold_pct = threshold_pct  # legacy compat
        self.pixel_diff = pixel_diff        # legacy compat

        # GMM state (initialized on first frame)
        self._gmm: list[list[GaussianComponent]] | None = None
        self._width = 0
        self._height = 0
        self._frame_count = 0

        # Trajectory tracker
        self._tracks: list[TrackedBlob] = []
        self._next_track_id = 1

        # Profile (resolved on first frame or when mode changes)
        self._profile = PROFILES["day"]
        self._area_thresh_pct = SENSITIVITY_AREA_MAP[self.sensitivity - 1]

        # Fallback for warmup period
        self._prev_gray: list[int] | None = None

    def feed(self, jpeg_bytes: bytes) -> float:
        """Feed a JPEG frame. Returns motion score 0-100.

        Returns 0 during GMM warmup (first ~200 frames), using legacy
        frame differencing as fallback.
        """
        gray = self._to_grayscale_sized(jpeg_bytes)
        if gray is None:
            return 0.0

        self._frame_count += 1

        # Initialize GMM on first frame
        if self._gmm is None:
            self._init_gmm(gray)
            self._prev_gray = gray
            return 0.0

        # Auto-detect lighting mode
        if self.lighting_mode == "auto":
            self._auto_detect_lighting(gray)

        # Update GMM and get foreground mask
        fg_mask = self._gmm_update(gray)

        # During warmup, use legacy frame diff as fallback
        if self._frame_count < GMM_WARMUP_FRAMES:
            score = self._legacy_score(gray)
            self._prev_gray = gray
            return score

        self._prev_gray = gray

        # Morphological cleanup
        fg_mask = self._morph_cleanup(fg_mask)

        # Blob detection
        blobs = self._detect_blobs(fg_mask)

        # Dust filter
        if self._profile.get("dust_filter", False) or self.dust_filter_enabled:
            blobs = self._filter_dust(blobs)

        # Trajectory tracking
        confirmed = self._update_tracks(blobs)

        # Compute score
        if not confirmed:
            return 0.0

        total_area = sum(b.area for b in confirmed)
        frame_area = self._width * self._height
        if frame_area == 0:
            return 0.0

        fg_pct = (total_area / frame_area) * 100
        raw_score = min((fg_pct / max(self._area_thresh_pct, 0.1)) * 50, 100)
        return raw_score

    def reset(self):
        """Clear all state."""
        self._gmm = None
        self._prev_gray = None
        self._tracks = []
        self._frame_count = 0

    @property
    def warmup_progress(self) -> float:
        """GMM warmup progress (0.0 to 1.0)."""
        return min(self._frame_count / GMM_WARMUP_FRAMES, 1.0)

    # ── GMM core ─────────────────────────────────────────────────────────

    def _init_gmm(self, gray: list[int]):
        """Initialize GMM state from first frame."""
        n_pixels = len(gray)
        sigma = self._profile.get("noise_sigma", GMM_NOISE_SIGMA)
        var_init = sigma * sigma

        self._gmm = []
        for px_val in gray:
            components = []
            for k in range(GMM_NUM_GAUSSIANS):
                if k == 0:
                    components.append(GaussianComponent(
                        mean=float(px_val),
                        variance=var_init,
                        weight=1.0,
                    ))
                else:
                    components.append(GaussianComponent(
                        mean=0.0,
                        variance=var_init,
                        weight=0.0,
                    ))
            self._gmm.append(components)

    def _gmm_update(self, gray: list[int]) -> list[int]:
        """Update GMM model and return foreground mask (0=bg, 1=fg)."""
        n = len(gray)
        fg_mask = [0] * n
        lr = self._profile.get("learning_rate", GMM_LEARNING_RATE)
        var_thresh = self._profile.get("var_thresh", GMM_VAR_THRESH)
        bg_ratio = GMM_BACKGROUND_RATIO
        sigma_init = self._profile.get("noise_sigma", GMM_NOISE_SIGMA)

        for i in range(n):
            x = float(gray[i])
            components = self._gmm[i]
            matched = False

            # Try to match against existing gaussians
            for g in components:
                if g.weight < 1e-6:
                    continue
                std = math.sqrt(max(g.variance, 1.0))
                if abs(x - g.mean) < var_thresh * std:
                    # Match — update
                    g.weight = (1.0 - lr) * g.weight + lr
                    rho = lr / max(g.weight, 1e-6)
                    g.mean = (1.0 - rho) * g.mean + rho * x
                    diff = x - g.mean
                    g.variance = (1.0 - rho) * g.variance + rho * diff * diff
                    g.variance = max(g.variance, 1.0)  # floor
                    matched = True
                    break
                else:
                    g.weight = (1.0 - lr) * g.weight

            if not matched:
                # Replace weakest gaussian
                weakest_idx = min(range(GMM_NUM_GAUSSIANS),
                                  key=lambda k: components[k].weight)
                components[weakest_idx] = GaussianComponent(
                    mean=x,
                    variance=sigma_init * sigma_init,
                    weight=GMM_WEIGHT_INIT,
                )

            # Normalize weights
            total_w = sum(g.weight for g in components)
            if total_w > 0:
                for g in components:
                    g.weight /= total_w

            # Classify: background or foreground?
            # Sort by weight/sqrt(variance) descending
            sorted_comps = sorted(
                components,
                key=lambda g: g.weight / math.sqrt(max(g.variance, 1.0)),
                reverse=True,
            )
            cumulative = 0.0
            is_bg = False
            for g in sorted_comps:
                cumulative += g.weight
                std = math.sqrt(max(g.variance, 1.0))
                if abs(x - g.mean) < var_thresh * std:
                    is_bg = True
                    break
                if cumulative > bg_ratio:
                    break

            fg_mask[i] = 0 if is_bg else 1

        return fg_mask

    # ── Morphological cleanup ────────────────────────────────────────────

    def _morph_cleanup(self, mask: list[int]) -> list[int]:
        """Erode then dilate to remove noise and reconnect regions."""
        w, h = self._width, self._height
        # Erode
        eroded = self._morph_op(mask, w, h, MORPH_ERODE_SIZE, op="erode")
        # Dilate
        dilated = self._morph_op(eroded, w, h, MORPH_DILATE_SIZE, op="dilate")
        return dilated

    @staticmethod
    def _morph_op(mask: list[int], w: int, h: int, ksize: int, op: str) -> list[int]:
        """Simple morphological operation (erode or dilate)."""
        out = list(mask)
        r = ksize // 2
        for y in range(h):
            for x in range(w):
                if op == "erode":
                    # Erode: output 1 only if ALL neighbors are 1
                    val = 1
                    for dy in range(-r, r + 1):
                        for dx in range(-r, r + 1):
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < h and 0 <= nx < w:
                                if mask[ny * w + nx] == 0:
                                    val = 0
                                    break
                            else:
                                val = 0
                                break
                        if val == 0:
                            break
                    out[y * w + x] = val
                else:
                    # Dilate: output 1 if ANY neighbor is 1
                    val = 0
                    for dy in range(-r, r + 1):
                        for dx in range(-r, r + 1):
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < h and 0 <= nx < w:
                                if mask[ny * w + nx] == 1:
                                    val = 1
                                    break
                        if val == 1:
                            break
                    out[y * w + x] = val
        return out

    # ── Blob detection ───────────────────────────────────────────────────

    def _detect_blobs(self, mask: list[int]) -> list[TrackedBlob]:
        """Connected component labeling via flood-fill."""
        w, h = self._width, self._height
        visited = [False] * (w * h)
        blobs: list[TrackedBlob] = []
        min_area = self._profile.get("min_blob_area", MIN_BLOB_AREA_PX)
        edge_margin = int(max(w, h) * EDGE_MARGIN_PCT / 100)

        for y in range(h):
            for x in range(w):
                idx = y * w + x
                if mask[idx] == 0 or visited[idx]:
                    continue

                # Flood-fill to find connected component
                stack = [(x, y)]
                pixels = []
                while stack:
                    px, py = stack.pop()
                    pidx = py * w + px
                    if visited[pidx]:
                        continue
                    visited[pidx] = True
                    if mask[pidx] == 0:
                        continue
                    pixels.append((px, py))
                    # 4-connected neighbors
                    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                        nx, ny = px + dx, py + dy
                        if 0 <= nx < w and 0 <= ny < h and not visited[ny * w + nx]:
                            stack.append((nx, ny))

                area = len(pixels)
                if area < min_area:
                    continue

                # Compute centroid
                cx = sum(p[0] for p in pixels) / area
                cy = sum(p[1] for p in pixels) / area

                # Edge filter: reject if centroid is near border
                if (cx < edge_margin or cx > w - edge_margin
                        or cy < edge_margin or cy > h - edge_margin):
                    continue

                blobs.append(TrackedBlob(cx=cx, cy=cy, area=area))

        return blobs

    # ── Dust/insect filter ───────────────────────────────────────────────

    def _filter_dust(self, blobs: list[TrackedBlob]) -> list[TrackedBlob]:
        """Filter out dust/insect blobs (small, near top of frame)."""
        ir_zone_y = self._height * DUST_IR_ZONE_TOP_PCT / 100
        return [
            b for b in blobs
            if not (b.area < DUST_MAX_AREA_PX and b.cy < ir_zone_y)
        ]

    # ── Trajectory tracker ───────────────────────────────────────────────

    def _update_tracks(self, blobs: list[TrackedBlob]) -> list[TrackedBlob]:
        """Match current blobs to tracked blobs, return confirmed ones."""
        matched_tracks = set()
        matched_blobs = set()

        # Match existing tracks to current blobs by nearest centroid
        for ti, track in enumerate(self._tracks):
            best_dist = TRAJECTORY_SEARCH_RADIUS_PX ** 2
            best_bi = -1
            for bi, blob in enumerate(blobs):
                if bi in matched_blobs:
                    continue
                dist = (track.cx - blob.cx) ** 2 + (track.cy - blob.cy) ** 2
                if dist < best_dist:
                    best_dist = dist
                    best_bi = bi

            if best_bi >= 0:
                # Update track with matched blob
                b = blobs[best_bi]
                track.cx = b.cx
                track.cy = b.cy
                track.area = b.area
                track.frames_seen += 1
                track.frames_missing = 0
                matched_tracks.add(ti)
                matched_blobs.add(best_bi)

        # Increment missing count for unmatched tracks
        for ti, track in enumerate(self._tracks):
            if ti not in matched_tracks:
                track.frames_missing += 1

        # Remove stale tracks
        self._tracks = [
            t for t in self._tracks
            if t.frames_missing <= TRAJECTORY_MAX_GAP_FRAMES
        ]

        # Start new tracks for unmatched blobs
        for bi, blob in enumerate(blobs):
            if bi not in matched_blobs:
                blob.id = self._next_track_id
                self._next_track_id += 1
                blob.frames_seen = 1
                self._tracks.append(blob)

        # Return confirmed tracks
        return [t for t in self._tracks if t.confirmed]

    # ── Auto lighting detection ──────────────────────────────────────────

    def _auto_detect_lighting(self, gray: list[int]):
        """Auto-detect lighting mode from frame brightness."""
        if not gray:
            return
        ## Brightness threshold for night detection.
        NIGHT_BRIGHTNESS_THRESHOLD = 50
        mean_brightness = sum(gray) / len(gray)
        if mean_brightness < NIGHT_BRIGHTNESS_THRESHOLD:
            self._profile = PROFILES.get("night_ir", PROFILES["day"])
        else:
            self._profile = PROFILES["day"]

    # ── Legacy frame-diff fallback (during warmup) ───────────────────────

    def _legacy_score(self, gray: list[int]) -> float:
        """Simple frame differencing (used during GMM warmup)."""
        prev = self._prev_gray
        if prev is None or len(prev) != len(gray):
            return 0.0
        return self._compute_score(prev, gray)

    def _compute_score(self, prev: list[int], curr: list[int]) -> float:
        """Compute percentage of pixels changed above threshold (legacy API)."""
        total = len(prev)
        if total == 0:
            return 0.0
        changed = sum(1 for p, c in zip(prev, curr)
                      if abs(p - c) > self.pixel_diff)
        return (changed / total) * 100.0

    # ── Image decoding ───────────────────────────────────────────────────

    @staticmethod
    def _to_grayscale(jpeg_bytes: bytes) -> list[int] | None:
        """Decode JPEG to flat grayscale pixel list (downsampled)."""
        try:
            from PIL import Image
            import io
            img = Image.open(io.BytesIO(jpeg_bytes))
            ratio = WORK_WIDTH / max(img.width, 1)
            new_h = max(int(img.height * ratio), 1)
            img = img.resize((WORK_WIDTH, new_h), Image.NEAREST)
            img = img.convert("L")
            return list(img.getdata())
        except ImportError:
            return None
        except Exception:
            return None

    def _to_grayscale_sized(self, jpeg_bytes: bytes) -> list[int] | None:
        """Decode JPEG and update width/height state."""
        gray = self._to_grayscale(jpeg_bytes)
        if gray is None:
            return None
        try:
            from PIL import Image
            import io
            img = Image.open(io.BytesIO(jpeg_bytes))
            ratio = WORK_WIDTH / max(img.width, 1)
            new_h = max(int(img.height * ratio), 1)
            self._width = WORK_WIDTH
            self._height = new_h
        except Exception:
            pass
        return gray
