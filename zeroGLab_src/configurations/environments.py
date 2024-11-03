import dataclasses
import os


@dataclasses.dataclass
class ZeroGLabConf:

    lab_length: float = 10.0
    lab_width: float = 6.5
    resolution: float = 0.01

    def __post_init__(self):
        assert type(self.resolution) == float, "The resolution must be a float."