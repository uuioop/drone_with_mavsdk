class SimpleMissionItem:
    """
    简单任务项类
    用于表示简单的任务项，包含纬度、经度、高度和速度。
    """
    def __init__(self, latitude, longitude, altitude, speed):
        if not (-90 <= latitude <= 90):
            raise ValueError("Latitude must be between -90 and 90 degrees.")
        if not (-180 <= longitude <= 180):
            raise ValueError("Longitude must be between -180 and 180 degrees.")
        if not (0 <= altitude <= 1000):
            raise ValueError("Altitude must be between 0 and 1000 meters.")
        if not (0 <= speed <= 20):
            raise ValueError("Speed must be between 0 and 20 m/s.")

        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.speed = speed