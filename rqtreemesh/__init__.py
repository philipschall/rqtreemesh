import math
from typing import Tuple

import numpy as np
from _rqtreemesh import generatemesh
from PIL import Image

Image.MAX_IMAGE_PIXELS = None

class Heightmap:
    """
    Provides heightmap raster to mesh conversion functionality.
    """
    def __init__(self, array: np.ndarray, pixel_dim: float, top_left_x: float,
                 top_left_y: float):
        """
        Create a Heightmap object by specifying geo-location properties.

        Parameters:
        ----------
        array : np.ndarray
            Input raster heightmap as a numpy array with shape (n, n), where
            n is a positive power of two.
        pixel_dim : float
            Spatial dimension of one pixel in the input heightmap.
        top_left_x : float
            The x coordinate of the top left corner of the input heightmap.
            Note that this location is not the center of the top left pixel,
            but its top left corner. The top left corner of the output mesh
            will be located here.
        top_left_y : float
            Similar to top_left_x, the y coordinate of the top left corner
            of the input heightmap.

        Raises:
        ------
        ValueError : If input array is not a 2D array with equal dimensions
                     a power of two.
        """
        if array.ndim != 2:
            raise ValueError("Input heighmap must be a 2D array.")
        if ((array.shape[0] != array.shape[1]) or
             not ((array.shape[0] & (array.shape[0] - 1) == 0) and
                   array.shape[0] != 0)):
            raise ValueError("""Both dimensions of input heightmap must be the
            same power of two.""")
        self.array = np.float32(array)
        self.max_depth = int(math.log2(array.shape[0]))
        self.pixel_dim = pixel_dim
        self.top_left_x = top_left_x
        self.top_left_y = top_left_y
    @ classmethod
    def from_geotiff(cls, path: str):
        """
        Create a Heightmap object by reading geo-location properties from a
        geoTIFF image.

        Parameters:
        ----------
        path : str
            Path to input raster heightmap in geoTIFF format of which the
            number of rows (and equally the number of columns) is a power
            of two. The geolocation and spatial resolution of the heightmap
            will be obtained from the appropriate tags in the geoTIFF.

        Raises:
        ------
        ValueError : If geoTIFF geo-location tags are not present, or image is
                     not single-channel.
        """
        img = Image.open(path)
        tags = img.tag_v2
        if not all(key in tags for key in [33550, 33922]):
            raise ValueError("""Input geoTIFF does not contain geo-location
            information.""")
        array = np.float32(img)
        if array.ndim != 2:
            raise ValueError("Input geoTIFF is multi-channel.")
        pixel_dim = tags[33550][0]
        top_left_x = tags[33922][3] - pixel_dim / 2
        top_left_y = tags[33922][4] + pixel_dim / 2
        return cls(array, pixel_dim, top_left_x, top_left_y)
    def generate_mesh(self, max_error: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate a 3D mesh using restricted qudtree triangulation.

        Parameters:
        ----------
        max_error : float
            The maximum absolute vertical distance between the input heightmap
            and the resulting mesh.

        Returns:
        -------
        output : Tuple[np.ndarray, np.ndarray]
            A tuple containing the output mesh.
            The first element is an array where each row contains the
            x, y and z coordinates of a vertex.
            The second element is a connectivity matrix, i.e. an array
            where each row contains a triangle as three (zero-indexed)
            indices into the first element.
        """
        return generatemesh(self.array, self.max_depth, max_error, self.pixel_dim,
                            self.top_left_x, self.top_left_y)

def write_obj(path: str, verts: np.ndarray, triangles: np.ndarray,
              flip_zy: bool = False):
    """
    Export a generated mesh to a .obj file.

    Parameters:
    ----------
    path : str
        Output file path.
    verts : np.ndarray
        A numpy array where each row contains the x, y and z coordinates of a
        vertex.
    triangles : np.ndarray
        A connectivity matrix as a numpy array, i.e each row contains a
        triangle as three (zero-indexed) indices into verts.
    flip_zy : bool
        Interchange the z and y axes of the exported mesh.
        Default value: False.
    """
    with open(path, 'w') as f:
        for vert in verts:
            f.write(" ".join(["v", str(vert[0]), str(vert[1 + flip_zy]),
                              str((1 - 2 * flip_zy) * vert[2 - flip_zy]),
                              "\n"]))
        f.write("\n")
        for triangle in triangles:
            f.write(" ".join(["f", str(int(triangle[0] + 1)),
                              str(int(triangle[1] + 1)),
                              str(int(triangle[2] + 1)), "\n"]))
