import abc
from typing import Tuple, Optional, Dict, Any
import numpy as np

class BaseFrameSource(abc.ABC):
    """
    Abstract interface for fetching frames.
    This ensures that downstream detection code never needs to know if the frame
    came from a synthetic rendering, Gazebo simulation, or a real Raspberry Pi camera.
    """
    
    @abc.abstractmethod
    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[Dict[str, Any]]]:
        """
        Fetches the next frame from the source.
        
        Returns:
            A tuple of (frame, pose_hint).
            - frame: A numpy ndarray representing the image (BGR format typically), 
                     or None if no frame is available.
            - pose_hint: Optional dictionary containing metadata about the frame, 
                         such as known camera pose (useful for testing synthetic frames)
                         or timestamps.
        """
        pass

    def close(self) -> None:
        """
        Cleans up any resources held by the frame source.
        """
        pass
