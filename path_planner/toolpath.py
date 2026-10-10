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
        
class LayeredToolPath:
    def __init__(self, base_path: ToolPath, layer_height: float, num_layers: int):
        if not type(num_layers) == int:
            raise ValueError("Number of layers must be an integer")

        if layer_height <= 0 or num_layers <= 0:
            raise ValueError("Layer height and number of layers must be positive")

        self.base_path = base_path
        self.layer_height = layer_height
        self.num_layers = num_layers

    def layer_z(self, layer_no: int):
        if layer_no < 0 or layer_no > self.num_layers + 1:
            raise ValueError("Layer number out of range")
        
        return self.layer_height * layer_no

    def layers(self):
        layers_heights = []
        for i in range(self.num_layers):
            layers_heights.append((self.layer_z(i), self.base_path.points.copy()))

        return layers_heights
