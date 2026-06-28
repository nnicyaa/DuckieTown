"""
Detects the leader's perforated marker bracket within a region of interest
(typically a YOLO-detected truck bounding box).

The bracket is a light-gray perforated mounting plate with a regular grid of
holes that appear dark/shadowed against the lighter bracket material. Rather
than detect and track each hole individually (which is noisy at distance,
where holes blur together or fall below a few pixels), this module:

    1. Crops to the search region (e.g. the YOLO truck bbox).
    2. Thresholds for dark, roughly-circular blobs within that crop.
    3. Requires at least `minimum_detected_dots` plausible holes before
       trusting the detection at all (avoids false positives from a single
       noisy dark spot -- shadows, screws, gaps in the chassis, etc).
    4. Fits one tight bounding box around the accepted hole cluster and
       returns that as the marker region, along with the hole count and
       cluster centroid.

This deliberately does NOT track individual hole identities frame-to-frame.
The bracket's geometry (rectangular grid) is fixed, so the cluster bounding
box and centroid are a stable, low-noise stand-in for "where the leader's
marker plate is and how big it appears" -- which is exactly what distance
and steering estimation need. Detecting individual holes precisely would add
complexity (correspondence between frames, partial occlusion handling)
without improving on what the cluster-level summary already gives.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

BBox = Tuple[int, int, int, int]


@dataclass
class MarkerDetection:
    found: bool
    bbox: Optional[BBox]          # cluster bounding box, in the ORIGINAL frame's coordinates
    center_x: Optional[float]
    center_y: Optional[float]
    dot_count: int
    reason: str
    dot_centers: Tuple[Tuple[float, float], ...] = ()
    used_plate_mask: bool = False


class MarkerBracketDetector:
    def __init__(
        self,
        minimum_detected_dots: int = 3,
        min_hole_radius: float = 1.5,
        max_hole_radius: float = 14.0,
        # Holes must be darker than this fraction of the crop's own
        # brightness range to count -- adaptive per-crop rather than a fixed
        # global threshold, since lighting on the bracket varies with
        # distance and ambient light.
        dark_threshold_ratio: float = 0.45,
        # After thresholding, reject crops whose dark-pixel ratio is wildly
        # outside what a perforated bracket should produce (helps reject
        # crops that are mostly shadow or mostly empty).
        min_dark_pixel_ratio: float = 0.01,
        max_dark_pixel_ratio: float = 0.55,
        # Cluster bbox is expanded slightly beyond the tight hole-cluster
        # extent, since the bracket's solid border around the holes is part
        # of the marker too and shouldn't be cropped out of the reported box.
        cluster_padding_ratio: float = 0.15,
        # White-plate pre-mask: the bracket itself is white/light gray,
        # mounted on a blue chassis. Before looking for dark holes, first
        # isolate "white-ish, low-saturation, bright" pixels via HSV -- this
        # restricts hole search to the bracket's own area, rejecting dark
        # blobs that fall on the surrounding blue chassis, cables, or
        # background (which would otherwise look like valid holes to a
        # plain brightness threshold). Set use_white_plate_mask=False to
        # fall back to searching the whole crop (e.g. if the bracket isn't
        # reliably white/light in your lighting).
        use_white_plate_mask: bool = True,
        white_min_value: int = 120,      # HSV V channel, 0-255
        white_max_saturation: int = 80,  # HSV S channel, 0-255
        # Minimum fraction of the crop that must be classified as "white
        # plate" for the pre-mask to be trusted -- if too little of the
        # crop looks like the bracket (e.g. truck is at a bad angle, or
        # mostly occluded), fall back to searching the unmasked crop rather
        # than searching a near-empty region and finding nothing.
        min_white_plate_ratio: float = 0.04,
    ):
        self.minimum_detected_dots = minimum_detected_dots
        self.min_hole_radius = min_hole_radius
        self.max_hole_radius = max_hole_radius
        self.dark_threshold_ratio = dark_threshold_ratio
        self.min_dark_pixel_ratio = min_dark_pixel_ratio
        self.max_dark_pixel_ratio = max_dark_pixel_ratio
        self.cluster_padding_ratio = cluster_padding_ratio

        self.use_white_plate_mask = use_white_plate_mask
        self.white_min_value = white_min_value
        self.white_max_saturation = white_max_saturation
        self.min_white_plate_ratio = min_white_plate_ratio

    def detect(
        self,
        frame_bgr: np.ndarray,
        search_bbox: BBox,
    ) -> MarkerDetection:
        """
        frame_bgr: full camera frame, BGR color order.

        IMPORTANT: this method only uses cv2.cvtColor(..., COLOR_BGR2GRAY)
        for grayscale conversion. If your caller's frame is actually RGB
        (e.g. real_server.py's `frame_rgb` after cv2.cvtColor(frame_bgr,
        COLOR_BGR2RGB)), passing it here directly will swap the red/blue
        channels during grayscale conversion -- not catastrophic for a
        brightness-threshold approach (grayscale luminance from R/G/B
        swapped channels is close but not identical), but it's a real
        correctness bug, not a no-op. Callers must either pass the original
        BGR frame, or convert back to BGR before calling this.

        search_bbox: region to search within, in frame_bgr's coordinates --
                     typically the YOLO truck detection's bbox, possibly with
                     some margin already applied by the caller.
        """
        x1, y1, x2, y2 = search_bbox
        h, w = frame_bgr.shape[:2]

        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))

        if x2 <= x1 or y2 <= y1:
            return self._not_found("invalid_search_region")

        crop = frame_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return self._not_found("empty_crop")

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        # White-plate pre-mask: the bracket is white/light gray, mounted on
        # a blue chassis. Before looking for dark holes, first isolate
        # "light, low-saturation" pixels via HSV -- this restricts hole
        # search to the bracket's own area, rejecting dark blobs that fall
        # on the surrounding blue chassis, cables, or background (which a
        # plain brightness threshold over the whole crop would otherwise
        # treat as candidate holes).
        plate_mask: Optional[np.ndarray] = None
        if self.use_white_plate_mask:
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            saturation = hsv[:, :, 1]
            value = hsv[:, :, 2]

            candidate_plate_mask = (
                (value >= self.white_min_value) & (saturation <= self.white_max_saturation)
            ).astype(np.uint8)

            # Close small gaps -- the dark holes themselves punch gaps in
            # this mask, so closing makes the plate read as one connected
            # region for the area-ratio check below. The dark threshold
            # step further down still finds the holes themselves; this
            # closing is only to stabilize "is there a plate-sized region
            # here at all."
            close_kernel = np.ones((5, 5), np.uint8)
            candidate_plate_mask = cv2.morphologyEx(candidate_plate_mask, cv2.MORPH_CLOSE, close_kernel)

            plate_ratio = float(np.count_nonzero(candidate_plate_mask)) / float(candidate_plate_mask.size)

            if plate_ratio >= self.min_white_plate_ratio:
                plate_mask = candidate_plate_mask
            # else: too little of the crop looks like the bracket (bad
            # angle, occlusion, lighting) -- fall through and search the
            # full crop below rather than searching an almost-empty mask.

        # Adaptive dark threshold: holes are darker than most of the
        # bracket, so use a percentile-based cut rather than a fixed value
        # -- this tracks ambient brightness changes automatically. When a
        # plate_mask is available, the percentile is computed only over
        # plate pixels, since mixing in dark chassis/background pixels
        # would skew the percentile and make the threshold too permissive.
        if plate_mask is not None:
            plate_pixel_values = gray[plate_mask > 0]
            if plate_pixel_values.size == 0:
                return self._not_found("plate_mask_empty_after_close")
            threshold_value = np.percentile(plate_pixel_values, self.dark_threshold_ratio * 100.0)
        else:
            threshold_value = np.percentile(gray, self.dark_threshold_ratio * 100.0)

        dark_mask = (gray <= threshold_value).astype(np.uint8) * 255

        if plate_mask is not None:
            # Only count/search dark pixels that fall within (or
            # immediately border) the plate region -- a hole has to be
            # surrounded by plate material to be a real hole, not just any
            # dark pixel anywhere in the crop.
            dilated_plate = cv2.dilate(plate_mask, np.ones((7, 7), np.uint8))
            dark_mask = cv2.bitwise_and(dark_mask, dark_mask, mask=dilated_plate)

        dark_ratio = float(np.count_nonzero(dark_mask)) / float(dark_mask.size)
        use_dark_contours = self.min_dark_pixel_ratio <= dark_ratio <= self.max_dark_pixel_ratio

        accepted_centers: List[Tuple[float, float]] = []

        if use_dark_contours:
            # Clean up speckle noise before contour extraction.
            kernel = np.ones((3, 3), np.uint8)
            dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area <= 0:
                    continue

                (cx, cy), radius = cv2.minEnclosingCircle(contour)

                if radius < self.min_hole_radius or radius > self.max_hole_radius:
                    continue

                # Roughly circular check: a true hole's contour area should be
                # reasonably close to a circle of that enclosing radius. This
                # rejects elongated shadows/cable shapes that happen to be dark
                # but aren't hole-shaped.
                circle_area = np.pi * radius * radius
                if circle_area <= 0:
                    continue
                circularity = area / circle_area
                if circularity < 0.35:
                    continue

                accepted_centers.append((cx, cy))

        if len(accepted_centers) < self.minimum_detected_dots:
            accepted_centers.extend(
                self._detect_edge_circles(gray, plate_mask, accepted_centers)
            )

        dot_count = len(accepted_centers)

        if dot_count < self.minimum_detected_dots:
            return self._not_found(
                f"insufficient_dots_detected={dot_count}_required={self.minimum_detected_dots}"
            )

        xs = [c[0] for c in accepted_centers]
        ys = [c[1] for c in accepted_centers]

        cluster_x1, cluster_x2 = min(xs), max(xs)
        cluster_y1, cluster_y2 = min(ys), max(ys)

        pad_x = (cluster_x2 - cluster_x1) * self.cluster_padding_ratio
        pad_y = (cluster_y2 - cluster_y1) * self.cluster_padding_ratio
        # Guard against a degenerate (near-zero) cluster extent collapsing
        # padding to nothing -- fall back to a small fixed pad in that case.
        pad_x = max(pad_x, 2.0)
        pad_y = max(pad_y, 2.0)

        crop_h, crop_w = gray.shape[:2]
        bx1 = max(0.0, cluster_x1 - pad_x)
        bx2 = min(float(crop_w), cluster_x2 + pad_x)
        by1 = max(0.0, cluster_y1 - pad_y)
        by2 = min(float(crop_h), cluster_y2 + pad_y)

        # Convert back to original frame coordinates.
        frame_bx1 = int(round(bx1 + x1))
        frame_bx2 = int(round(bx2 + x1))
        frame_by1 = int(round(by1 + y1))
        frame_by2 = int(round(by2 + y1))

        center_x = float(np.mean(xs)) + x1
        center_y = float(np.mean(ys)) + y1
        dot_centers = tuple((float(cx + x1), float(cy + y1)) for cx, cy in accepted_centers)

        return MarkerDetection(
            found=True,
            bbox=(frame_bx1, frame_by1, frame_bx2, frame_by2),
            center_x=center_x,
            center_y=center_y,
            dot_count=dot_count,
            reason="marker_bracket_detected",
            dot_centers=dot_centers,
            used_plate_mask=plate_mask is not None,
        )

    @staticmethod
    def _is_duplicate_center(
        center: Tuple[float, float],
        existing_centers: List[Tuple[float, float]],
        min_distance: float,
    ) -> bool:
        cx, cy = center
        for ex, ey in existing_centers:
            dx = cx - ex
            dy = cy - ey
            if (dx * dx + dy * dy) ** 0.5 < min_distance:
                return True
        return False

    def _detect_edge_circles(
        self,
        gray: np.ndarray,
        plate_mask: Optional[np.ndarray],
        existing_centers: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """
        Fallback for simulated/painted circular dots that are not dark holes
        on a white plate. It uses circle edges instead of dark-pixel blobs.
        """
        min_radius = max(1, int(round(self.min_hole_radius)))
        max_radius = max(min_radius + 1, int(round(self.max_hole_radius)))
        min_dist = max(4.0, self.min_hole_radius * 2.5)

        blurred = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=min_dist,
            param1=80,
            param2=10,
            minRadius=min_radius,
            maxRadius=max_radius,
        )

        if circles is None:
            return []

        accepted: List[Tuple[float, float]] = []
        mask_for_filter = None
        if plate_mask is not None:
            mask_for_filter = cv2.dilate(plate_mask, np.ones((7, 7), np.uint8))

        for cx, cy, radius in np.round(circles[0, :]).astype("int"):
            if radius < min_radius or radius > max_radius:
                continue
            if mask_for_filter is not None:
                if cy < 0 or cy >= mask_for_filter.shape[0] or cx < 0 or cx >= mask_for_filter.shape[1]:
                    continue
                if mask_for_filter[cy, cx] == 0:
                    continue
            center = (float(cx), float(cy))
            if self._is_duplicate_center(center, existing_centers + accepted, min_dist * 0.5):
                continue
            accepted.append(center)

        return accepted

    @staticmethod
    def _not_found(reason: str) -> MarkerDetection:
        return MarkerDetection(
            found=False,
            bbox=None,
            center_x=None,
            center_y=None,
            dot_count=0,
            reason=reason,
            dot_centers=(),
        )
