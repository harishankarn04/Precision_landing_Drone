import cv2
import numpy as np
from typing import Dict, Any, List, Optional
from sim.board import TAGS

class TagFusion:
    def __init__(self, disagreement_threshold_m=0.15):
        self.disagreement_threshold_m = disagreement_threshold_m
        
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
