import numpy as np

class ToolPath:
    def __init__(self, points: list, speed: float):
        if not self._if_speed_valid(speed):
            raise ValueError("Speed must be positive")

        if not self._if_coord_valid(*points):
            raise ValueError("Invalid coordinates: points cannot be below ground")
        
        self.points = points 
        self.speed = speed

    @staticmethod
    def _if_speed_valid(value : float):
        return value > 0

    @staticmethod
    def _if_coord_valid(*value: tuple):
        for x, y in value:
            if y < 0:
                return False
        return True

    @classmethod
    def line(cls, start: tuple, end: tuple, speed: float):
        points = [start, end]
        return cls(points, speed)
    
    @classmethod
    def rectangle(cls, centre: tuple, width: float, height: float, speed: float):
        if width <= 0 or height <= 0:
            raise ValueError("Width and height must be positive")
        
        cx, cy = centre
        half_width = width / 2
        half_height = height / 2

        points = [(cx - half_width, cy - half_height),(cx + half_width, cy - half_height), (cx + half_width, cy + half_height), (cx - half_width, cy + half_height), (cx - half_width, cy - half_height)]
        return cls(points, speed)

    @classmethod
    def square(cls, centre: tuple, side_length: float, speed: float):
        if side_length <= 0:
            raise ValueError("Side length must be positive")
        
        return cls.rectangle(centre, side_length, side_length, speed)

    @classmethod
    def triangle(cls, centre: tuple, side_length: float, speed: float):
        if side_length <= 0:
            raise ValueError("Side length must be positive")
        
        cx, cy = centre
        height = np.sqrt(side_length ** 2 - (side_length / 2) ** 2)
        half_width = side_length / 2

        points = [(cx, cy + (2 / 3) * height), (cx + half_width, cy - (1 / 3) * height), (cx - half_width, cy - (1 / 3) * height), (cx, cy + (2 / 3) * height)]
        return cls(points, speed)
    
    @classmethod
    def ellipse(cls, centre: tuple, height: float, width: float, speed: float):
        if height <= 0 or width <= 0:
            raise ValueError("Radius must be positive")
        
        cx, cy = centre
        angles  = np.linspace(0, 2 * np.pi, 100)

        x = cx + (width / 2) * np.cos(angles)
        y = cy + (height / 2) * np.sin(angles)

        points = list(zip(x, y))
        return cls(points, speed)

    @classmethod
    def circle(cls, centre: tuple, radius: float, speed: float):
        if radius <= 0:
            raise ValueError("Radius must be positive")
        
        diameter = 2 * radius
        return cls.ellipse(centre, diameter, diameter, speed)
        
class LayeredToolPath

    