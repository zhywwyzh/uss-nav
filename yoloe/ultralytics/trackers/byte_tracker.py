# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

from __future__ import annotations

import time
from typing import Any

import numpy as np

from ..utils import LOGGER
from ..utils.ops import xywh2ltwh
from .basetrack import BaseTrack, TrackState
from .utils import matching
from .utils.kalman_filter import KalmanFilterXYAH
from .utils.stracks import joint_stracks, merge_track_pools, multi_gmc, parse_bboxes


class STrack(BaseTrack):
    """Single object tracking representation that uses Kalman filtering for state estimation.

    This class is responsible for storing all the information regarding individual tracklets and performs state updates
    and predictions based on Kalman filter.

    Attributes:
        shared_kalman (KalmanFilterXYAH): Shared Kalman filter used across all STrack instances for prediction.
        _tlwh (np.ndarray): Private attribute to store top-left corner coordinates and width and height of bounding box.
        kalman_filter (KalmanFilterXYAH): Instance of Kalman filter used for this particular object track.
        mean (np.ndarray): Mean state estimate vector.
        covariance (np.ndarray): Covariance of state estimate.
        is_activated (bool): Boolean flag indicating if the track has been activated.
        score (float): Confidence score of the track.
        tracklet_len (int): Length of the tracklet.
        cls (Any): Class label for the object.
        idx (int): Index or identifier for the object.
        frame_id (int): Current frame ID.
        start_frame (int): Frame where the object was first detected.
        angle (float | None): Optional angle information for oriented bounding boxes.

    Methods:
        predict: Predict the next state of the object using Kalman filter.
        multi_predict: Predict the next states for multiple tracks.
        activate: Activate a new tracklet.
        re_activate: Reactivate a previously lost tracklet.
        update: Update the state of a matched track.
        convert_coords: Convert bounding box to x-y-aspect-height format.
        tlwh_to_xyah: Convert tlwh bounding box to xyah format.

    Examples:
        Initialize and activate a new track
        >>> track = STrack(xywh=[100, 200, 50, 80, 0], score=0.9, cls="person")
        >>> track.activate(kalman_filter=KalmanFilterXYAH(), frame_id=1)
    """

    shared_kalman = KalmanFilterXYAH()

    def __init__(self, xywh: np.ndarray, score: float, cls: Any):
        """Initialize a new STrack instance.

        Args:
            xywh (np.ndarray): Bounding box in `(x, y, w, h, idx)` or `(x, y, w, h, angle, idx)` format, where (x, y) is
                the center, (w, h) are width and height, and `idx` is the detection index.
            score (float): Confidence score of the detection.
            cls (Any): Class label for the detected object.
        """
        super().__init__()
        # xywh+idx or xywha+idx
        assert len(xywh) in {5, 6}, f"expected 5 or 6 values but got {len(xywh)}"
        self._tlwh = np.asarray(xywh2ltwh(xywh[:4]), dtype=np.float32)
        self.kalman_filter = None
        self.mean, self.covariance = None, None
        self.is_activated = False

        self.score = score
        self.tracklet_len = 0
        self.cls = cls
        self.idx = xywh[-1]
        self.angle = xywh[4] if len(xywh) == 6 else None

    def predict(self):
        """Predict the next state (mean and covariance) of the object using the Kalman filter."""
        mean_state = self.mean.copy()
        if self.state != TrackState.Tracked:
            mean_state[7] = 0
        self.mean, self.covariance = self.kalman_filter.predict(mean_state, self.covariance)

    @staticmethod
    def multi_predict(stracks: list[STrack]):
        """Perform multi-object predictive tracking using Kalman filter for the provided list of STrack instances."""
        if not stracks:
            return
        multi_mean = np.asarray([st.mean.copy() for st in stracks])
        multi_covariance = np.asarray([st.covariance for st in stracks])
        for i, st in enumerate(stracks):
            if st.state != TrackState.Tracked:
                multi_mean[i][7] = 0
        multi_mean, multi_covariance = STrack.shared_kalman.multi_predict(multi_mean, multi_covariance)
        for i, (mean, cov) in enumerate(zip(multi_mean, multi_covariance)):
            stracks[i].mean = mean
            stracks[i].covariance = cov

    def activate(self, kalman_filter: KalmanFilterXYAH, frame_id: int):
        """Activate a new tracklet using the provided Kalman filter and initialize its state and covariance."""
        self.kalman_filter = kalman_filter
        self.track_id = self.next_id()
        self.mean, self.covariance = self.kalman_filter.initiate(self.convert_coords(self._tlwh))

        self.tracklet_len = 0
        self.state = TrackState.Tracked
        if frame_id == 1:
            self.is_activated = True
        self.frame_id = frame_id
        self.start_frame = frame_id

    def re_activate(self, new_track: STrack, frame_id: int, new_id: bool = False):
        """Reactivate a previously lost track using new detection data and update its state and attributes."""
        self.mean, self.covariance = self.kalman_filter.update(
            self.mean, self.covariance, self.convert_coords(new_track.tlwh)
        )
        self.tracklet_len = 0
        self.state = TrackState.Tracked
        self.is_activated = True
        self.frame_id = frame_id
        if new_id:
            self.track_id = self.next_id()
        self.score = new_track.score
        self.cls = new_track.cls
        self.angle = new_track.angle
        self.idx = new_track.idx

    def update(self, new_track: STrack, frame_id: int):
        """Update the state of a matched track.

        Args:
            new_track (STrack): The new track containing updated information.
            frame_id (int): The ID of the current frame.

        Examples:
            Update the state of a track with new detection information
            >>> track = STrack([100, 200, 50, 80, 0], score=0.9, cls=0)
            >>> new_track = STrack([105, 205, 55, 85, 0], score=0.95, cls=0)
            >>> track.update(new_track, 2)
        """
        self.frame_id = frame_id
        self.tracklet_len += 1

        new_tlwh = new_track.tlwh
        self.mean, self.covariance = self.kalman_filter.update(
            self.mean, self.covariance, self.convert_coords(new_tlwh)
        )
        self.state = TrackState.Tracked
        self.is_activated = True

        self.score = new_track.score
        self.cls = new_track.cls
        self.angle = new_track.angle
        self.idx = new_track.idx

    def convert_coords(self, tlwh: np.ndarray) -> np.ndarray:
        """Convert a bounding box's top-left-width-height format to its x-y-aspect-height equivalent."""
        return self.tlwh_to_xyah(tlwh)

    @property
    def tlwh(self) -> np.ndarray:
        """Get the bounding box in top-left-width-height format from the current state estimate."""
        if self.mean is None:
            return self._tlwh.copy()
        ret = self.mean[:4].copy()
        ret[2] *= ret[3]
        ret[:2] -= ret[2:] / 2
        return ret

    @property
    def xyxy(self) -> np.ndarray:
        """Convert bounding box from (top left x, top left y, width, height) to (min x, min y, max x, max y) format."""
        ret = self.tlwh  # already a fresh array, safe to mutate
        ret[2:] += ret[:2]
        return ret

    @staticmethod
    def tlwh_to_xyah(tlwh: np.ndarray) -> np.ndarray:
        """Convert bounding box from tlwh format to center-x-center-y-aspect-height (xyah) format."""
        ret = np.asarray(tlwh).copy()
        ret[:2] += ret[2:] / 2
        ret[2] /= ret[3]
        return ret

    @property
    def xywh(self) -> np.ndarray:
        """Get the current position of the bounding box in (center x, center y, width, height) format."""
        ret = np.asarray(self.tlwh).copy()
        ret[:2] += ret[2:] / 2
        return ret

    @property
    def xywha(self) -> np.ndarray:
        """Get position in (center x, center y, width, height, angle) format, warning if angle is missing."""
        if self.angle is None:
            LOGGER.warning("`angle` attr not found, returning `xywh` instead.")
            return self.xywh
        return np.concatenate([self.xywh, self.angle[None]])

    @property
    def result(self) -> list[float]:
        """Get the current tracking results in the appropriate bounding box format."""
        coords = self.xyxy if self.angle is None else self.xywha
        return [*coords.tolist(), self.track_id, self.score, self.cls, self.idx]

    def __repr__(self) -> str:
        """Return a string representation of the STrack object including start frame, end frame, and track ID."""
        return f"OT_{self.track_id}_({self.start_frame}-{self.end_frame})"


class BYTETracker:
    """BYTETracker: A tracking algorithm built on top of YOLO for object detection and tracking.

    This class encapsulates the functionality for initializing, updating, and managing the tracks for detected objects
    in a video sequence. It maintains the state of tracked, lost, and removed tracks over frames, utilizes Kalman
    filtering for predicting the new object locations, and performs data association.

    Attributes:
        tracked_stracks (list[STrack]): List of successfully activated tracks.
        lost_stracks (list[STrack]): List of lost tracks.
        removed_stracks (list[STrack]): List of removed tracks.
        frame_id (int): The current frame ID.
        args (Namespace): Command-line arguments.
        max_frames_lost (int): The maximum frames for a track to be considered as 'lost'.
        kalman_filter (KalmanFilterXYAH): Kalman Filter object.

    Methods:
        update: Update object tracker with new detections.
        get_kalmanfilter: Return a Kalman filter object for tracking bounding boxes.
        init_track: Initialize object tracking with detections.
        get_dists: Calculate the distance between tracks and detections.
        multi_predict: Predict the location of tracks.
        reset_id: Reset the ID counter of STrack.
        reset: Reset the tracker by clearing all tracks.
        joint_stracks: Combine two lists of stracks.
        sub_stracks: Filter out the stracks present in the second list from the first list.
        remove_duplicate_stracks: Remove duplicate stracks based on IoU.

    Examples:
        Initialize BYTETracker and update with detection results
        >>> tracker = BYTETracker(args)
        >>> results = yolo_model.detect(image)
        >>> tracked_objects = tracker.update(results)
    """

    track_class = STrack

    def __init__(self, args):
        """Initialize a BYTETracker instance for object tracking.

        Args:
            args (Namespace): Command-line arguments containing tracking parameters.
        """
        self.tracked_stracks: list[STrack] = []
        self.lost_stracks: list[STrack] = []
        self.removed_stracks: list[STrack] = []

        self.frame_id = 0
        self.args = args
        self.max_frames_lost = args.track_buffer
        self.max_lost_stracks = max(1, int(getattr(args, "max_lost_stracks", 100)))
        self.max_lost_seconds = max(0.0, float(getattr(args, "max_lost_seconds", 3.0)))
        self.kalman_filter = self.get_kalmanfilter()
        self.reset_id()

    def update(self, results, img: np.ndarray | None = None, feats: np.ndarray | None = None, **kwargs) -> np.ndarray:
        """Update the tracker with new detections and return the current list of tracked objects."""
        self.frame_id += 1
        self._update_stamp = float(kwargs.get("stamp", time.monotonic()))
        # 必须在关联前淘汰超龄对象，否则本帧复活后会绕过末尾超龄检查。
        expired = []
        self._remove_stale_lost(expired)
        self.lost_stracks = [t for t in self.lost_stracks if t.state == TrackState.Lost]
        self.removed_stracks = (self.removed_stracks + expired)[-1000:]
        self.oru_gap = self.oru_steps = 0
        self.oru_ms = 0.0
        activated_stracks = []
        refind_stracks = []
        lost_stracks = []
        removed_stracks = []

        results_high, results_low, mask_high, mask_low = self._split_detections(results)
        detections = self.init_track(results_high, self._input_for(img, feats, mask_high))
        encoder = getattr(self, "encoder", None)
        high_times = dict(getattr(encoder, "timings", {})) if len(results_high) else {}
        detections_second = self.init_track(results_low, self._input_for(img, feats, mask_low))
        low_times = dict(getattr(encoder, "timings", {})) if len(results_low) else {}
        self.reid_timings = {key: high_times.get(key, 0) + low_times.get(key, 0)
                             for key in high_times.keys() | low_times.keys()}
        for tracks, mask in ((detections, mask_high), (detections_second, mask_low)):
            for track, i in zip(tracks, np.flatnonzero(mask)):
                track.idx = i  # idx must be in full detection-set space; parse_bboxes only sees the subset

        unconfirmed, tracked_stracks = self._split_tracked()
        strack_pool = self._build_strack_pool(tracked_stracks)
        self.multi_predict(strack_pool)
        self._pre_first_associate(strack_pool, unconfirmed, img, results_high)

        u_track, u_detection = self._first_association(strack_pool, detections, activated_stracks, refind_stracks)
        u_track, u_detection = self._post_first_association(
            strack_pool, detections, u_track, u_detection, activated_stracks, refind_stracks
        )
        self._second_association(
            strack_pool, u_track, detections_second, activated_stracks, refind_stracks, lost_stracks
        )
        u_detection, detections = self._unconfirmed_association(
            unconfirmed, u_detection, detections, activated_stracks, removed_stracks
        )
        self._init_new_tracks(u_detection, detections, activated_stracks, refind_stracks)
        self._remove_stale_lost(removed_stracks)
        merge_track_pools(self, activated_stracks, refind_stracks, lost_stracks, removed_stracks)
        # 合并新 lost 后才限制容量，保证本帧新丢失对象也计入上限。
        evicted = []
        self._enforce_lost_cap(evicted)
        self.lost_stracks = [t for t in self.lost_stracks if t.state == TrackState.Lost]
        self.removed_stracks = (self.removed_stracks + evicted)[-1000:]
        for track in self.tracked_stracks:
            track.last_observed_stamp = self._update_stamp
        return self._format_output()

    def _split_detections(self, results: Any) -> tuple[Any, Any, np.ndarray, np.ndarray]:
        """Split detections into high-confidence and low-confidence subsets.

        Args:
            results (Any): Results-like object with ``conf`` attribute supporting boolean indexing.

        Returns:
            (tuple[Any, Any, np.ndarray, np.ndarray]): High-confidence results, low-confidence results, high mask, and
                low mask.
        """
        scores = results.conf
        remain_inds = scores >= self.args.track_high_thresh
        inds_low = scores > self.args.track_low_thresh
        inds_below_high = scores < self.args.track_high_thresh
        return results[remain_inds], results[inds_low & inds_below_high], remain_inds, inds_low & inds_below_high

    def _input_for(self, img: np.ndarray | None, feats: np.ndarray | None, mask: np.ndarray) -> Any:
        """Return the per-detection auxiliary input for ``init_track``.

        When ``feats`` is provided it is sliced by the detection mask. Trackers with a native
        (``model="auto"``) ReID encoder get None when feats are missing (e.g. user-supplied
        detections), so ``init_track`` falls back to the no-encoding path instead of feeding the
        BGR frame into the auto encoder. External ReID models always take the frame.

        Args:
            img (np.ndarray | None): Current BGR frame.
            feats (np.ndarray | None): Optional per-detection features.
            mask (np.ndarray): Boolean mask used to slice ``feats``.

        Returns:
            (Any): The auxiliary payload (features, image or None) to hand to ``init_track``.
        """
        if feats is not None and len(feats):
            return feats[mask]
        if getattr(self, "encoder", None) is not None and getattr(self.args, "model", "auto") == "auto":
            return None
        return img

    def _split_tracked(self) -> tuple[list[STrack], list[STrack]]:
        """Separate ``self.tracked_stracks`` into confirmed and unconfirmed lists.

        Returns:
            (tuple[list[STrack], list[STrack]]): ``(unconfirmed, tracked)`` where ``unconfirmed`` holds tracks whose
                ``is_activated`` flag is False.
        """
        unconfirmed, tracked = [], []
        for track in self.tracked_stracks:
            (unconfirmed if not track.is_activated else tracked).append(track)
        return unconfirmed, tracked

    def _pre_first_associate(
        self, strack_pool: list[STrack], unconfirmed: list[STrack], img: np.ndarray | None, results_high: Any
    ) -> None:
        """Hook called after Kalman predict, before first-stage assignment. Default: GMC if available."""
        if hasattr(self, "gmc") and img is not None:
            try:
                warp = self.gmc.apply(img, results_high.xyxy)
            except Exception as e:
                LOGGER.warning(f"GMC failed, falling back to identity: {e}")
                warp = np.eye(2, 3)
            multi_gmc(strack_pool, warp)
            multi_gmc(unconfirmed, warp)

    def _first_association(
        self, strack_pool: list[STrack], detections: list[STrack], activated: list[STrack], refind: list[STrack]
    ) -> tuple[list[int], list[int]]:
        """First-stage association between track pool and high-score detections.

        Returns:
            (tuple[list[int], list[int]]): Unmatched track indices and unmatched detection indices.
        """
        dists = self.get_dists(strack_pool, detections)
        matches, u_track, u_detection = matching.linear_assignment(dists, thresh=self.args.match_thresh)
        self._apply_matches(matches, strack_pool, detections, activated, refind)
        return u_track, u_detection

    def _post_first_association(
        self,
        strack_pool: list[STrack],
        detections: list[STrack],
        u_track: list[int],
        u_detection: list[int],
        activated: list[STrack],
        refind: list[STrack],
    ) -> tuple[list[int], list[int]]:
        """Hook executed after the first association stage and before the second.

        Returns:
            (tuple[list[int], list[int]]): Potentially modified unmatched track and detection indices.
        """
        return u_track, u_detection

    def _apply_matches(
        self,
        matches: list[list[int]] | np.ndarray,
        pool: list[STrack],
        detections: list[STrack],
        activated: list[STrack],
        refind: list[STrack],
    ) -> None:
        """Apply a list of matched (track, detection) pairs from an association stage."""
        for itracked, idet in matches:
            self._apply_match(pool[itracked], detections[idet], activated, refind)

    def _apply_match(self, track: STrack, det: STrack, activated: list[STrack], refind: list[STrack]) -> None:
        """Update or re-activate a single track with its matched detection."""
        if track.state == TrackState.Tracked:
            track.update(det, self.frame_id)
            activated.append(track)
        else:
            track.re_activate(det, self.frame_id, new_id=False)
            refind.append(track)

    def _second_association(
        self,
        strack_pool: list[STrack],
        u_track: list[int],
        detections_second: list[STrack],
        activated: list[STrack],
        refind: list[STrack],
        lost: list[STrack],
    ) -> None:
        """Second-stage association between remaining tracked tracks and low-score detections."""
        r_tracked_stracks = [strack_pool[i] for i in u_track if strack_pool[i].state == TrackState.Tracked]
        if r_tracked_stracks and detections_second:
            # IoU-only by design (ByteTrack paper sec. 3.2): fusing low scores pushes costs above the 0.5 threshold
            dists = matching.iou_distance(r_tracked_stracks, detections_second)
            matches, u_track, _ = matching.linear_assignment(dists, thresh=0.5)
            self._apply_matches(matches, r_tracked_stracks, detections_second, activated, refind)
        else:
            u_track = list(range(len(r_tracked_stracks)))

        for it in u_track:
            track = r_tracked_stracks[it]
            if track.state != TrackState.Lost:
                track.mark_lost()
                lost.append(track)

    def _unconfirmed_association(
        self,
        unconfirmed: list[STrack],
        u_detection: list[int],
        detections: list[STrack],
        activated: list[STrack],
        removed: list[STrack],
    ) -> tuple[list[int], list[STrack]]:
        """Associate unconfirmed tracks with leftover high-score detections.

        Returns:
            (tuple[list[int], list[STrack]]): Unmatched detection indices after association, and the filtered detection
                list those indices refer to.
        """
        detections = [detections[i] for i in u_detection]
        if not unconfirmed:
            return list(range(len(detections))), detections
        dists = self.get_dists(unconfirmed, detections)
        matches, u_unconfirmed, u_detection = matching.linear_assignment(dists, thresh=0.7)
        for itracked, idet in matches:
            unconfirmed[itracked].update(detections[idet], self.frame_id)
            activated.append(unconfirmed[itracked])
        for it in u_unconfirmed:
            track = unconfirmed[it]
            track.mark_removed()
            removed.append(track)
        return u_detection, detections

    def _init_new_tracks(
        self,
        u_detection: list[int],
        detections: list[STrack],
        activated: list[STrack],
        refind: list[STrack] | None = None,
    ) -> None:
        """Activate new tracks from detections that survived all association stages."""
        for inew in u_detection:
            track = detections[inew]
            if track.score < self.args.new_track_thresh:
                continue
            track.activate(self.kalman_filter, self.frame_id)
            activated.append(track)

    def _remove_stale_lost(self, removed: list[STrack]) -> None:
        """Remove lost tracks that have exceeded the maximum allowed frames."""
        for track in self.lost_stracks:
            age_seconds = self._update_stamp - getattr(track, "last_observed_stamp", self._update_stamp)
            if (self.frame_id - track.end_frame > self.max_frames_lost
                    or (self.max_lost_seconds > 0 and age_seconds > self.max_lost_seconds)):
                track.mark_removed()
                removed.append(track)

    def _build_strack_pool(self, tracked_stracks: list[STrack]) -> list[STrack]:
        """构造关联池：Tracked + Lost；Removed 状态一律排除，不参与任何匹配。"""
        lost_pool = [t for t in self.lost_stracks if t.state == TrackState.Lost]
        return joint_stracks(tracked_stracks, lost_pool)

    def _enforce_lost_cap(self, removed: list[STrack]) -> None:
        """数量保护：lost 池超过 max_lost_stracks 时只淘汰最旧（end_frame 最小）的 Lost 轨迹。

        在年龄上限以内优先保留 protected_track_id，其他对象按观测时间淘汰；
        被淘汰对象本帧即标记 Removed 并在 merge_track_pools 中离开 lost 池。
        """
        cap = self.max_lost_stracks
        if cap <= 0:
            return
        candidates = [t for t in self.lost_stracks if t.state == TrackState.Lost]
        if len(candidates) <= cap:
            return
        evict = sorted(candidates, key=lambda t: (t.track_id == getattr(self, "protected_track_id", None), t.end_frame))[: len(candidates) - cap]
        for track in evict:
            track.mark_removed()
            removed.append(track)

    def get_diagnostics(self) -> dict:
        """返回池规模与年龄诊断（供服务端结构化日志使用，只读无副作用）。"""
        max_lost_age = max((self.frame_id - t.end_frame for t in self.lost_stracks), default=0)
        return {
            "tracked": len(self.tracked_stracks),
            "lost": len(self.lost_stracks),
            "removed": len(self.removed_stracks),
            "max_lost_age_frames": int(max_lost_age),
            "frame_id": int(self.frame_id),
            "oru_gap": getattr(self, "oru_gap", 0),
            "oru_steps": getattr(self, "oru_steps", 0),
            "oru_ms": getattr(self, "oru_ms", 0.0),
        }

    def _format_output(self) -> np.ndarray:
        """Format the current tracked objects into the output array."""
        return np.asarray([x.result for x in self.tracked_stracks if x.is_activated], dtype=np.float32)

    def get_kalmanfilter(self) -> KalmanFilterXYAH:
        """Return a Kalman filter object for tracking bounding boxes using KalmanFilterXYAH."""
        return KalmanFilterXYAH()

    def init_track(self, results, img: np.ndarray | None = None) -> list[STrack]:
        """Initialize object tracking with given detections, scores, and class labels as STrack instances."""
        if len(results) == 0:
            return []
        bboxes = parse_bboxes(results)
        return [self.track_class(xywh, s, c) for (xywh, s, c) in zip(bboxes, results.conf, results.cls)]

    def get_dists(self, tracks: list[STrack], detections: list[STrack]) -> np.ndarray:
        """Calculate the distance between tracks and detections using IoU and optionally fuse scores."""
        dists = matching.iou_distance(tracks, detections)
        if self.args.fuse_score:
            dists = matching.fuse_score(dists, detections)
        return dists

    def multi_predict(self, tracks: list[STrack]):
        """Predict the next states for multiple tracks using Kalman filter."""
        STrack.multi_predict(tracks)

    @staticmethod
    def reset_id():
        """Reset the ID counter for STrack instances to ensure unique track IDs across tracking sessions."""
        STrack.reset_id()

    def reset(self):
        """Reset the tracker by clearing all tracked, lost, and removed tracks and reinitializing the Kalman filter."""
        self.tracked_stracks = []
        self.lost_stracks = []
        self.removed_stracks = []
        self.frame_id = 0
        self.kalman_filter = self.get_kalmanfilter()
        self.reset_id()
