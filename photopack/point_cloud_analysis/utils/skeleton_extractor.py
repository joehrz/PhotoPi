import os
import time
import pickle
import numpy as np
import random

from skeleton_utils import (
    get_boxes, find_all_connections, count_boxes_with_points, count_connections,
    plot_boxes, draw_cubes, make_dim_list, find_all_Vpairs, find_all_Epairs,
    still_vpairs, eat_all_Vpairs, eat_one_epair
)
from mybox import myBox

class SkeletonExtractor:
    """
    A class for extracting skeletons from point clouds using the SkelTree algorithm.
    This class encapsulates the entire workflow:
    
    1. Load points from a NumPy array.
    2. Subdivide into boxes.
    3. Create a dictionary of boxes (myDict).
    4. Find connections between boxes (forming an octree-like structure).
    5. Perform collapsing operations (Vpairs, Epairs) to produce a skeleton.
    6. Optionally save the dictionary for later use.
    """

    def __init__(self, nboxes=5000, save_dict=False, save_folder="./Data", save_name="myDict", use_higher_dimensional_boxes=False):
        """
        Initialize the SkeletonExtractor.

        Parameters
        ----------
        nboxes : int
            Number of boxes in which to divide the bounding box of the given set of points.
        save_dict : bool
            Whether to save the final dictionary of boxes as a pickle file.
        save_folder : str
            Folder to save the dictionary file.
        save_name : str
            Base name for the saved dictionary file.
        use_higher_dimensional_boxes : bool
            Whether to attempt connecting boxes in higher octree levels if no connections are found at the lowest level.
        """
        self.nboxes = nboxes
        self.save_dict = save_dict
        self.save_folder = os.path.abspath(save_folder)
        self.save_name = save_name
        self.use_higher_dimensional_boxes = use_higher_dimensional_boxes

        self.points = None
        self.boxes = None
        self.myDict = {}

    def load_points(self, points):
        """
        Load points from a NumPy array.

        Parameters
        ----------
        points : np.ndarray
            N x 3 array of points.
        """
        if not isinstance(points, np.ndarray) or points.shape[1] != 3:
            raise ValueError("Points must be an Nx3 NumPy array.")

        self.points = points
        print(f"Loaded {len(points)} points.")

    def create_boxes(self):
        """
        Subdivide the space containing the points into N boxes and initialize them.
        """
        if self.points is None:
            raise ValueError("Points not loaded. Please load points before creating boxes.")

        print(f"Creating ~{self.nboxes} boxes for {len(self.points)} points!")
        t0 = time.perf_counter()
        self.boxes = get_boxes(self.nboxes, self.points)
        t1 = time.perf_counter()
        print("\tTime:", round(t1-t0,3),"seconds!")

        # Shuffle boxes to avoid bias
        random.shuffle(self.boxes)

        # Create box objects
        for box_tuple in self.boxes:
            points_box = box_tuple[0]
            box_name = box_tuple[1]
            self.myDict[box_name] = myBox(self.myDict, box_name, points_box, use_higher_dimensional_boxes=self.use_higher_dimensional_boxes)

    def build_octree(self, threshold_list=[16]):
        """
        Build an octree-like structure by finding connections between boxes.

        Parameters
        ----------
        threshold_list : list of int
            List of thresholds (as denominators) to try (e.g., [16] means threshold=1/16).
        """
        for t in threshold_list:
            threshold = 1/t
            print(f"Getting octtree using threshold 1/{t}...")
            t0 = time.perf_counter()
            find_all_connections(self.myDict, threshold)
            t1 = time.perf_counter()

            number_of_boxes = count_boxes_with_points(self.myDict)
            number_of_connections = count_connections(self.myDict)
            print(f"\tFound {number_of_boxes} boxes with points!")
            print(f"\tTotal connections: {number_of_connections}!")
            print("\tTime:", round(t1-t0,3),"seconds!")

            cg, labels_cg = plot_boxes(self.myDict)
            cube_points, cube_labels = draw_cubes(self.boxes)

            print("Performing collapsing procedure...")
            self.collapse_skeleton([5,4,3,2])  # Example dimensions

            cg, labels_cg = plot_boxes(self.myDict)

            # Save results if requested
            if self.save_dict:
                self.save_dictionary(t)

    def collapse_skeleton(self, dims=[5,4,3,2]):
        """
        Collapses the skeleton by merging boxes (using Vpairs and Epairs).

        Parameters
        ----------
        dims : list of int
            The dimensions (like 5,4,3,2) used during the collapsing procedure.
        """
        t0 = time.perf_counter()

        for dim in dims:
            dim_list = make_dim_list(self.myDict, dim)
            find_all_Vpairs(self.myDict, dim_list)
            find_all_Epairs(self.myDict, dim_list)

            vpairs_there = True
            cycles = 0

            while vpairs_there:
                _ = eat_all_Vpairs(self.myDict, dim_list)
                find_all_Vpairs(self.myDict, dim_list)
                vpairs_there, total = still_vpairs(self.myDict, dim_list)

                cycles += 1
                if vpairs_there:
                    # Continue searching for Vpairs
                    continue
                else:
                    find_all_Epairs(self.myDict, dim_list)
                    success = eat_one_epair(self.myDict, dim_list)
                    find_all_Vpairs(self.myDict, dim_list)

                    if success:
                        vpairs_there = True
            print(f"\tFinished dim {dim} collapse in {cycles} cycles!")

        t1 = time.perf_counter()
        print("Collapse Time:", round(t1-t0,3),"seconds!")


    def convert_skeleton_to_graph(self):
        """
        Convert the final skeleton (myDict) into a graph representation:
        - node_positions: Nx3 array of node coordinates
        - edges: Mx2 array of edges (undirected)

        Returns
        -------
        node_positions : np.ndarray
            N x 3 array containing positions of each skeleton node.
        edges : np.ndarray
            M x 2 array containing edges between nodes.
        """
        # Filter out merged boxes or boxes without points to get the final nodes
        node_names = [name for name, box in self.myDict.items() if (not box.merged) and box.contains_points]

        # Map each node (box) name to an index
        node_index_map = {}
        node_positions = []

        for i, name in enumerate(node_names):
            node_index_map[name] = i
            box = self.myDict[name]
            node_positions.append(box.cg)

        node_positions = np.array(node_positions)

        # Build edge list
        edges_list = []
        visited_pairs = set()

        for name in node_names:
            box = self.myDict[name]
            u = node_index_map[name]
            for conn_name in box.connections.keys():
                if conn_name in node_index_map:
                    v = node_index_map[conn_name]
                    # Ensure each edge is added only once
                    edge_pair = tuple(sorted((u, v)))
                    if edge_pair not in visited_pairs:
                        visited_pairs.add(edge_pair)
                        edges_list.append(edge_pair)

        edges = np.array(edges_list)

        return node_positions, edges







    def save_dictionary(self, t):
        """
        Save the final dictionary of boxes as a pickle file.

        Parameters
        ----------
        t : int
            The threshold denominator used (for naming the file).
        """
        if not os.path.isdir(self.save_folder):
            os.mkdir(self.save_folder)

        name_file = f"{self.save_name}_N{str(self.nboxes)[0:2]}K_t{t}.pkl"
        file_path = os.path.join(self.save_folder, name_file)
        with open(file_path, "wb") as f:
            pickle.dump(self.myDict, f, pickle.HIGHEST_PROTOCOL)
        print(f"Saved the dict!\n\tLocation: {file_path}")
