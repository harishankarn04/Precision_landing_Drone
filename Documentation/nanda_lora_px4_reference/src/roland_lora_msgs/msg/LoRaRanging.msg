std_msgs/Header header
int32 anchor_id        # ID of the LoRa anchor (0-3)
int32 tag_id           # ID of the LoRa tag on the drone
float64 range          # Estimated distance in meters
float64 rssi           # Received Signal Strength Indicator (dBm)
float64 snr            # Signal-to-Noise Ratio (dB)
float64 range_error    # Estimated standard deviation of range (meters)
