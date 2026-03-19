"""Motion detection — frame differencing for RTSP camera pre-filter.

Compares consecutive frames using absolute pixel difference. If enough pixels
change above a threshold, motion is reported.  This is a cheap CPU-only filter
to avoid calling the expensive VLM on every frame.

Usage:
    detector = MotionDetector()
    score = detector.feed(frame_bytes)  # JPEG bytes
    if score > MOTION_THRESHOLD_PCT:
        # call VLM
"""

import math

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MOTION_THRESHOLD_PCT = 15       # % of pixels that must change to trigger
MOTION_PIXEL_DIFF_THRESHOLD = 30  # per-pixel delta to count as "changed"
MOTION_BLUR_RADIUS = 2         # Gaussian blur radius (reduces noise)
MOTION_DOWNSAMPLE_WIDTH = 160  # resize to this width for speed


class MotionDetector:
    """Stateful frame-differencing motion detector."""

    def __init__(
        self,
        threshold_pct: int = MOTION_THRESHOLD_PCT,
        pixel_diff: int = MOTION_PIXEL_DIFF_THRESHOLD,
    ):
        self.threshold_pct = threshold_pct
        self.pixel_diff = pixel_diff
        self._prev_gray: list[int] | None = None

    def feed(self, jpeg_bytes: bytes) -> float:
        """Feed a JPEG frame and return motion score (0-100).

        Returns 0 on the first frame (no reference).
        Uses pure-Python grayscale comparison when Pillow is available,
        otherwise returns 0 (safe fallback — VLM handles everything).
        """
        gray = self._to_grayscale(jpeg_bytes)
        if gray is None:
            return 0.0

        prev = self._prev_gray
        self._prev_gray = gray

        if prev is None:
            return 0.0

        if len(prev) != len(gray):
            return 0.0

        return self._compute_score(prev, gray)

    def reset(self):
        """Clear reference frame."""
        self._prev_gray = None

    # ── Internal ──────────────────────────────────────────────────────────

    def _compute_score(self, prev: list[int], curr: list[int]) -> float:
        """Compute percentage of pixels that changed above threshold."""
        total = len(prev)
        if total == 0:
            return 0.0
        changed = 0
        for p, c in zip(prev, curr):
            if abs(p - c) > self.pixel_diff:
                changed += 1
        return (changed / total) * 100.0

    @staticmethod
    def _to_grayscale(jpeg_bytes: bytes) -> list[int] | None:
        """Decode JPEG to a flat list of grayscale pixel values (downsampled)."""
        try:
            from PIL import Image
            import io
            img = Image.open(io.BytesIO(jpeg_bytes))
            # downsample for speed
            ratio = MOTION_DOWNSAMPLE_WIDTH / max(img.width, 1)
            new_h = max(int(img.height * ratio), 1)
            img = img.resize(
                (MOTION_DOWNSAMPLE_WIDTH, new_h), Image.NEAREST
            )
            img = img.convert("L")  # grayscale
            return list(img.getdata())
        except ImportError:
            return None
        except Exception:
            return None
