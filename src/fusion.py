import time
import cv2
import numpy as np
from typing import Dict, Any, List, Optional
from sim.board import TAGS

class TagFusion:
    def __init__(self, disagreement_threshold_m=0.15, loss_debounce_s=0.5):
        """
        loss_debounce_s: coast on the last good fused result for up to this many
        seconds of missed frames before actually reporting the target as lost --
        matches a teammate's independent reference project (github.com/format37/
        courierquad, DetectorCfg.loss_debounce_s), which measured the same real
        frame-to-frame detection flicker we saw (their notes: "~50-80% hit rate at
        5-8m") and found that reacting to every single miss as "lost" caused far more
        disruption than the flicker itself. Found 2026-09-24 after a live Gazebo test
        showed correction starting then stalling/losing the target mid-descent.
        """
        self.disagreement_threshold_m = disagreement_threshold_m
        self.loss_debounce_s = loss_debounce_s
        self._last_good = None
        self._last_good_t = 0.0

    def _camera_to_body_frame(self, cam_x, cam_y, cam_z):
        """
        Converts camera frame coordinates to vehicle body frame.
        Assuming camera is mounted facing exactly down.
        OpenCV camera: +X right, +Y down (image space), +Z forward (out of lens).
        Vehicle FRD (Forward-Right-Down): +X forward, +Y right, +Z down.
        So:
        Body X (forward) = - Camera Y (up)
        Body Y (right)   = + Camera X (right)
        Body Z (down)    = + Camera Z
        """
        body_x = -cam_y
        body_y = cam_x
        dist = cam_z
        return body_x, body_y, dist

    def fuse_tags(self, results: Dict[int, Any], corners: np.ndarray, ids: np.ndarray, image_shape) -> Optional[Dict[str, Any]]:
        """
        Public entry point: computes this frame's fresh fusion, then applies the
        coast/debounce described in __init__ -- a miss this frame doesn't immediately
        report "lost" if a good result landed within loss_debounce_s.
        """
        fresh = self._compute_fresh(results, corners, ids, image_shape)
        now = time.time()
        if fresh is not None:
            self._last_good = fresh
            self._last_good_t = now
            return fresh
        if self._last_good is not None and (now - self._last_good_t) <= self.loss_debounce_s:
            return self._last_good
        return None

    def _compute_fresh(self, results: Dict[int, Any], corners: np.ndarray, ids: np.ndarray, image_shape) -> Optional[Dict[str, Any]]:
        """
        results: dict from detector tag_id -> (rvec, tvec)
        corners: raw corners from aruco
        ids: raw ids from aruco
        """
        if not results or ids is None:
            return None
            
        h, w = image_shape[:2]
        margin = 10 # 10 pixels from edge
        
        valid = []
        for i, tag_id in enumerate(ids.flatten()):
            if tag_id not in results:
                continue
                
            rvec, tvec = results[tag_id]
            tag = TAGS[tag_id]
            cam_x, cam_y, cam_z = tvec.flatten()
            
            # Edge check
            tag_corners = corners[i].reshape(4, 2)
            x_min, y_min = tag_corners[:, 0].min(), tag_corners[:, 1].min()
            x_max, y_max = tag_corners[:, 0].max(), tag_corners[:, 1].max()
            
            edge_distance = min(x_min, y_min, w - x_max, h - y_max)
            edge_safe = edge_distance > margin
            
            # offset vector from landing center TO tag, in board plane
            offset_in_tag_frame = np.array([tag.offset_x_m, tag.offset_y_m, 0.0], dtype=float)
            
            # Rotate offset into camera frame
            R_tag, _ = cv2.Rodrigues(rvec)
            offset_in_cam = R_tag @ offset_in_tag_frame
            
            # Landing center = tag position - offset
            landing_cam_x = cam_x - offset_in_cam[0]
            landing_cam_y = cam_y - offset_in_cam[1]
            landing_cam_z = cam_z - offset_in_cam[2]
            
            body_x, body_y, dist = self._camera_to_body_frame(
                landing_cam_x, landing_cam_y, landing_cam_z
            )
            
            pixel_area = cv2.contourArea(tag_corners)
            
            # Weights
            area_weight = np.sqrt(pixel_area)
            edge_weight = 1.0 if edge_safe else 0.3
            weight = area_weight * edge_weight
            
            valid.append({
                'id': tag_id,
                'body_x': body_x,
                'body_y': body_y,
                'dist': dist,
                'weight': weight
            })
            
        if not valid:
            return None
            
        # Weighted average
        total_weight = sum(v['weight'] for v in valid)
        if total_weight > 0:
            avg_body_x = sum(v['body_x'] * v['weight'] for v in valid) / total_weight
            avg_body_y = sum(v['body_y'] * v['weight'] for v in valid) / total_weight
            avg_dist   = sum(v['dist']   * v['weight'] for v in valid) / total_weight
        else:
            avg_body_x = valid[0]['body_x']
            avg_body_y = valid[0]['body_y']
            avg_dist   = valid[0]['dist']
            
        # Disagreement check
        disagreement = 0.0
        if len(valid) > 1:
            for v in valid:
                dx = v['body_x'] - avg_body_x
                dy = v['body_y'] - avg_body_y
                d_err = float(np.sqrt(dx*dx + dy*dy))
                if d_err > disagreement:
                    disagreement = d_err
                    
        if disagreement > self.disagreement_threshold_m:
            # Reject frame if tags disagree strongly (e.g. false positive or severe artifact)
            return None
            
        return {
            'body_x': avg_body_x,
            'body_y': avg_body_y,
            'dist': avg_dist,
            'disagreement': disagreement,
            'tags_used': len(valid)
        }
