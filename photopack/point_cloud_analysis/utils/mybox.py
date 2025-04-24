import numpy as np

class myBox:
    """
    Class representing a box (vertex) in the SkelTree algorithm.
    Each box holds points, calculates its center of gravity (cg),
    and can find/connect to its neighbors, merge with other boxes, etc.
    """

    def calc_cg(self):
        self.cg = np.mean(self.points, axis=0)

    def __init__(self, parent_dict, box_name, points, use_higher_dimensional_boxes=False):
        """
        Initialize a box object.

        Parameters
        ----------
        parent_dict : dict
            Reference to the parent dictionary containing all box objects.
        box_name : str
            Name of this box object (format: 'Box_x_y_z').
        points : np.ndarray
            The points contained inside this box (Nx3 array).
        use_higher_dimensional_boxes : bool
            If True, tries to connect higher dimensional octree boxes 
            when no connections are found at the lowest level.
        """
        if not box_name.startswith("Box"):
            raise Exception("box_name should be of format: 'Box_x_y_z'")

        if not isinstance(points, np.ndarray):
            raise Exception("Points should be a NumPy ndarray Nx3.")

        if points.any():
            if points.shape[1] != 3:
                raise Exception("Points should be an Nx3 ndarray.")

        self.parent_dict = parent_dict
        self.name = box_name 
        self.points = points
        self.use_higher_dimensional_boxes = use_higher_dimensional_boxes

        # If more than 1 point
        if points.any():
            self.calc_cg()
            self.contains_points = True
        else:
            self.cg = None 
            self.contains_points = False

        self.merged = False
        self.parent = None
        self.children = []
        self.connections = {}
        self.Vdim = 0
        self.Vdir = []
        self.Vpairs = []
        self.Epairs = []

    def merged_with(self, box_name):
        """
        Sets this box to merged status, clearing all data.
        """
        self.parent = box_name
        self.merged = True

        self.points = np.array([])
        self.contains_points = False
        self.cg = None
        
        self.connections = {}
        self.children = []
        self.Vdim = 0
        self.Vdir = []
        self.Vpairs = []
        self.Epairs = []

    def replace_connections(self, parent):
        """
        Replace all connections to this box with the parent box.

        Any box connected to this box now gets connected to the parent box,
        provided the parent is not already connected. Also removes references 
        to this box from those boxes' Vpairs and Epairs.
        """
        for connection in list(self.connections.keys()):
            if connection == parent:
                continue

            box = self.get_box_object(connection)
            if not box:
                continue

            if parent not in box.connections:
                box.connections[parent] = box.connections[self.name]

            if self.name in box.connections:
                box.connections.pop(self.name)

            if self.name in box.Vpairs:
                box.Vpairs.remove(self.name)

            if self.name in box.Epairs:
                box.Epairs.remove(self.name)

    def calc_Vdim(self):
        distinct_labels = []
        for connection in self.connections:
            label = self.connections[connection]
            present = any((label == dl).all() for dl in distinct_labels)
            if not present:
                distinct_labels.append(label)
        self.Vdim = len(distinct_labels)

    def calc_Vdir(self):
        distinct_labels = []
        for connection in self.connections:
            label = self.connections[connection]
            present = any((label == dl).all() for dl in distinct_labels)
            if not present:
                distinct_labels.append(label)

        Vdir = [0, 0, 0]
        for label in distinct_labels:
            Vdir[0] += label[0]
            Vdir[1] += label[1]
            Vdir[2] += label[2]

        self.Vdir = Vdir
        
    def get_potential_neighbours_names(self):
        name_split = self.name.split("_")
        x = int(name_split[-3])
        y = int(name_split[-2])
        z = int(name_split[-1])

        neighbours = [
            'Box_' + str(x+1) + "_" + str(y) + "_" + str(z),
            'Box_' + str(x-1) + "_" + str(y) + "_" + str(z),
            'Box_' + str(x) + "_" + str(y+1) + "_" + str(z),
            'Box_' + str(x) + "_" + str(y-1) + "_" + str(z),
            'Box_' + str(x) + "_" + str(y) + "_" + str(z+1),
            'Box_' + str(x) + "_" + str(y) + "_" + str(z-1)
        ]
        return neighbours  

    def get_surrounding_boxes(self, box_name):
        """
        Gets all the boxes of an enlarged cube (i.e. one octree subdivision 
        level higher than the current box).
        """
        name_split = box_name.split("_")
        x_base = int(name_split[-3])
        y_base = int(name_split[-2])
        z_base = int(name_split[-1])
        
        x, y, z = np.mgrid[x_base-1:x_base+2:1, y_base-1:y_base+2:1, z_base-1:z_base+2:1]
        xyz_stack = np.vstack((x.flatten(), y.flatten(), z.flatten())).T

        neighbours = []
        for xyz_coord in xyz_stack:
            neighbour_name = 'Box_' + str(xyz_coord[0]) + "_" + str(xyz_coord[1]) + "_" + str(xyz_coord[2])
            if neighbour_name in self.parent_dict:
                neighbours.append(neighbour_name)
        
        if self.name in neighbours:
            neighbours.remove(self.name)
        return neighbours  

    def get_directional_labels(self, neighbours):
        name_box = self.name.split("_")
        x = int(name_box[-3])
        y = int(name_box[-2])
        z = int(name_box[-1])

        labels = {}
        for neighbour in neighbours:
            name_neighbour = neighbour.split("_")
            x_n = int(name_neighbour[-3])
            y_n = int(name_neighbour[-2])
            z_n = int(name_neighbour[-1])

            labels[neighbour] = np.array([x_n - x, y_n - y, z_n - z])
        return labels

    def get_box_object(self, box_name):
        return self.parent_dict.get(box_name, False)

    def get_neighbour_names(self):
        neighbour_names = self.get_potential_neighbours_names()
        return [n for n in neighbour_names if n in self.parent_dict]

    def calc_median_distance(self, points, cg, normal_vec):
        d = np.dot(normal_vec, cg)
        distances = [(np.dot(normal_vec, p) - d)**2 for p in points]
        return np.mean(distances)

    def check_connection_criteria(self, c1, c2, points1, points2, threshold):
        c12 = c1 + (c2 - c1)*0.5
        normal_plane_vec = (c2 - c1) / np.linalg.norm((c2 - c1))

        d1 = self.calc_median_distance(points1, c1, normal_plane_vec)
        d2 = self.calc_median_distance(points2, c2, normal_plane_vec)
        d12 = self.calc_median_distance(np.concatenate((points1, points2), axis=0), c12, normal_plane_vec)

        return (threshold * d12 <= min(d1, d2))

    def find_super_boxes(self, levels_higher):
        name_split = self.name.split("_")
        x_base = int(name_split[-3])
        y_base = int(name_split[-2])
        z_base = int(name_split[-1])

        if levels_higher == 1:
            x, y, z = np.mgrid[x_base-1:x_base+1:1, y_base:y_base+2:1, z_base:z_base+2:1]
        elif levels_higher == 2:
            x, y, z = np.mgrid[x_base-1:x_base+3:1, y_base-2:y_base+2:1, z_base-2:z_base+2:1]

        xyz_stack = np.vstack((x.flatten(), y.flatten(), z.flatten())).T
        super_box = []
        for coord in xyz_stack:
            box_name = "Box_" + str(coord[0]) + "_" + str(coord[1]) + "_" + str(coord[2])
            super_box.append(box_name)

        return super_box

    def find_adjacent_super_boxes(self, super_box, levels_higher):
        x_vals = []
        y_vals = []
        z_vals = []

        for box_name in super_box:
            name_split = box_name.split("_")
            x = int(name_split[-3])
            y = int(name_split[-2])
            z = int(name_split[-1])

            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z)

        x_min, x_max = min(x_vals), max(x_vals)
        y_min, y_max = min(y_vals), max(y_vals)
        z_min, z_max = min(z_vals), max(z_vals)

        step_size = (2*levels_higher)
        x, y, z = np.mgrid[x_min - step_size:x_max + step_size + 1:1, 
                           y_min - step_size:y_max + step_size + 1:1, 
                           z_min - step_size:z_max + step_size + 1:1]
        xyz_stack = np.vstack((x.flatten(), y.flatten(), z.flatten())).T

        adj_super_boxes = {
            'adj_super_box1': {'directional_label': [], 'box_names': []},
            'adj_super_box2': {'directional_label': [], 'box_names': []},
            'adj_super_box3': {'directional_label': [], 'box_names': []},
            'adj_super_box4': {'directional_label': [], 'box_names': []},
            'adj_super_box5': {'directional_label': [], 'box_names': []},
            'adj_super_box6': {'directional_label': [], 'box_names': []}
        }

        for xv, yv, zv in xyz_stack:
            box_name = "Box_" + str(xv) + "_" + str(yv) + "_" + str(zv)
            # Extended from x-axis
            if (xv < x_min or xv > x_max) and (yv >= y_min and yv <= y_max) and (zv >= z_min and zv <= z_max):
                if xv < x_min:
                    adj_super_boxes['adj_super_box1']['box_names'].append(box_name)
                    adj_super_boxes['adj_super_box1']['directional_label'] = np.array([-1, 0, 0])
                elif xv > x_max:
                    adj_super_boxes['adj_super_box2']['box_names'].append(box_name)
                    adj_super_boxes['adj_super_box2']['directional_label'] = np.array([1, 0, 0])

            # Extended from y-axis
            if (yv < y_min or yv > y_max) and (xv >= x_min and xv <= x_max) and (zv >= z_min and zv <= z_max):
                if yv < y_min:
                    adj_super_boxes['adj_super_box3']['box_names'].append(box_name)
                    adj_super_boxes['adj_super_box3']['directional_label'] = np.array([0, -1, 0])
                elif yv > y_max:
                    adj_super_boxes['adj_super_box4']['box_names'].append(box_name)
                    adj_super_boxes['adj_super_box4']['directional_label'] = np.array([0, 1, 0])

            # Extended from z-axis
            if (zv < z_min or zv > z_max) and (yv >= y_min and yv <= y_max) and (xv >= x_min and xv <= x_max):
                if zv < z_min:
                    adj_super_boxes['adj_super_box5']['box_names'].append(box_name)
                    adj_super_boxes['adj_super_box5']['directional_label'] = np.array([0, 0, -1])
                elif zv > z_max:
                    adj_super_boxes['adj_super_box6']['box_names'].append(box_name)
                    adj_super_boxes['adj_super_box6']['directional_label'] = np.array([0, 0, 1])

        return adj_super_boxes

    def get_super_box_properties(self, super_box):
        points = np.array([])
        for box in super_box:
            box_obj = self.get_box_object(box)
            if box_obj and box_obj.contains_points:
                if points.any():
                    points = np.concatenate((points, box_obj.points), axis=0)
                else:
                    points = box_obj.points

        if points.any():
            cg = np.mean(points, axis=0)
        else:
            cg = np.array([0,0,0])

        return points, cg

    def get_best_connection_to_super_box(self, super_box, directional_label):
        distances = []
        potential_boxes = []
        for b in super_box:
            box_obj = self.get_box_object(b)
            if box_obj and box_obj.contains_points:
                distance_vec = (self.cg - box_obj.cg)
                distance = np.dot(distance_vec, distance_vec)
                distances.append(distance)
                potential_boxes.append(box_obj.name)

        if distances:
            index_min = np.argmin(distances)
            box_to_connect = potential_boxes[index_min]
            min_distance = distances[index_min]

            box_obj = self.get_box_object(box_to_connect)
            self.connections[box_obj.name] = directional_label
            box_obj.connections[self.name] = -directional_label

            return min_distance, box_to_connect
        else:
            return False, False

    def find_connections_higher_level_box(self, threshold):
        levels_higher = 1
        while len(self.connections) == 0:
            super_box = self.find_super_boxes(levels_higher)
            adj_super_boxes = self.find_adjacent_super_boxes(super_box, levels_higher)
            points1, cg1 = self.get_super_box_properties(super_box)

            possible_connection = {}
            for sb_name in adj_super_boxes:
                adj_sb_names = adj_super_boxes[sb_name]['box_names']
                directional_label = adj_super_boxes[sb_name]['directional_label']

                points2, cg2 = self.get_super_box_properties(adj_sb_names)

                if points2.any():
                    connection = self.check_connection_criteria(cg1, cg2, points1, points2, threshold)
                else:
                    connection = False

                if connection:
                    dist, b_name = self.get_best_connection_to_super_box(adj_sb_names, directional_label)
                    if b_name:
                        possible_connection[b_name] = [dist, directional_label]

            if levels_higher == 2:
                break
            levels_higher += 1

    def find_connections(self, threshold):
        neighbour_names = self.get_neighbour_names()
        directional_labels = self.get_directional_labels(neighbour_names)

        for n_name in neighbour_names:
            neighbour = self.get_box_object(n_name)
            if not neighbour or neighbour.name in self.connections or neighbour.contains_points == False:
                continue

            connection = self.check_connection_criteria(self.cg, neighbour.cg, self.points, neighbour.points, threshold)
            if connection:
                directional_label = directional_labels[neighbour.name]
                self.connections[neighbour.name] = directional_label
                neighbour.connections[self.name] = -directional_label
                neighbour.calc_Vdim()
                neighbour.calc_Vdir()

        if self.use_higher_dimensional_boxes and len(self.connections) < 2 and self.contains_points:
            self.find_connections_higher_level_box(threshold)

        self.calc_Vdim()
        self.calc_Vdir()

    def get_combined_dim(self, neighbour):
        total_connections = self.connections.copy()
        if neighbour.name in total_connections:
            total_connections.pop(neighbour.name)

        for c in neighbour.connections.keys():
            if c != self.name and c not in total_connections:
                total_connections[c] = neighbour.connections[c]

        distinct_labels = []
        for connection in total_connections.keys():
            label = total_connections[connection]
            present = any((label == dl).all() for dl in distinct_labels)
            if not present:
                distinct_labels.append(label)

        combined_dim = len(distinct_labels)
        return combined_dim

    def find_v_pairs(self):
        found_pair = False
        for neighbour_name in self.connections.keys():
            neighbour = self.get_box_object(neighbour_name)
            if not neighbour:
                continue

            if neighbour_name in self.Vpairs:
                continue

            combined_dim = self.get_combined_dim(neighbour)
            if not (combined_dim <= max(neighbour.Vdim, self.Vdim)):
                continue

            # Check for identical neighbor with matching direction
            for cn in neighbour.connections.keys():
                if cn in self.connections.keys():
                    if (neighbour.connections[cn] == self.connections[cn]).all():
                        self.Vpairs.append(neighbour_name)
                        found_pair = True
        return found_pair

    def find_e_pairs(self):
        found_pair = False
        for neighbour_name in self.connections.keys():
            neighbour = self.get_box_object(neighbour_name)
            if not neighbour or neighbour.merged:
                continue
            if neighbour_name in self.Epairs:
                continue

            combined_dim = self.get_combined_dim(neighbour)
            if not (combined_dim <= max(neighbour.Vdim, self.Vdim)):
                continue

            if self.Vdir.count(0) == 3:
                continue

            connection_direction = self.connections[neighbour_name]
            nz_conn_dir = np.nonzero(connection_direction)[0]
            nz_vdir = np.nonzero(self.Vdir)[0]

            # Check directional alignment
            if len(nz_conn_dir) == 0 or not (nz_conn_dir[0] in nz_vdir):
                continue

            # Check line formation
            if len(self.connections) < 3 or len(neighbour.connections) < 3:
                continue

            self.Epairs.append(neighbour.name)
            found_pair = True

        return found_pair

    def get_best_epair(self, box_name):
        box = self.get_box_object(box_name)
        if not box:
            return "", 999

        smallest_norm = 999
        best_Epair = ""

        for Epair_name in box.Epairs:
            Epair_obj = self.get_box_object(Epair_name)
            if Epair_obj and not Epair_obj.merged:
                norm = abs(Epair_obj.Vdir[0]) + abs(Epair_obj.Vdir[1]) + abs(Epair_obj.Vdir[2])
                if norm < smallest_norm:
                    best_Epair = Epair_obj.name
                    smallest_norm = norm
            else:
                # Remove merged box from Epairs
                if Epair_name in box.Epairs:
                    box.Epairs.remove(Epair_name)

        return best_Epair, smallest_norm

    def get_best_vpair(self, box_name, requester_name):
        box = self.get_box_object(box_name)
        if not box:
            return "", 999

        smallest_norm = 999
        best_Vpair = ""

        for Vpair_name in box.Vpairs:
            if Vpair_name == requester_name:
                continue
            Vpair_obj = self.get_box_object(Vpair_name)
            if Vpair_obj and not Vpair_obj.merged:
                norm = abs(Vpair_obj.Vdir[0]) + abs(Vpair_obj.Vdir[1]) + abs(Vpair_obj.Vdir[2])
                if norm < smallest_norm:
                    best_Vpair = Vpair_obj.name
                    smallest_norm = norm
            else:
                if Vpair_name in box.Vpairs:
                    box.Vpairs.remove(Vpair_name)

        return best_Vpair, smallest_norm

    def eat_v_pair(self):
        ate_vpair = False
        for Vpair_name in self.Vpairs:
            self.eat_box(Vpair_name)
            ate_vpair = True
            break
        return ate_vpair

    def eat_e_pair(self):
        ate_Epair = False
        best_Epair, norm = self.get_best_epair(self.name)
        if best_Epair:
            box = self.get_box_object(best_Epair)
            if not box:
                return False

            box_best_epair, box_norm = box.get_best_epair(box.name)
            if norm <= box_norm or box_best_epair == self.name:
                ate_Epair = True
                self.eat_box(box.name)
        return ate_Epair

    def eat_box(self, box_name):
        box_to_eat = self.get_box_object(box_name)
        if not box_to_eat:
            return False

        # Add points
        if box_to_eat.contains_points:
            self.points = np.concatenate((self.points, box_to_eat.points), axis=0)
        # Add children
        self.children.append(box_to_eat.name)
        for ch in box_to_eat.children:
            self.children.append(ch)

        # Recalculate CG
        self.calc_cg()

        # Add connections
        for c in box_to_eat.connections.keys():
            if c == self.name:
                continue
            if c not in self.connections:
                self.connections[c] = box_to_eat.connections[c]

        box_to_eat.replace_connections(self.name)
        if box_name in self.connections:
            self.connections.pop(box_name)

        if box_name in self.Vpairs:
            self.Vpairs.remove(box_name)

        if box_name in self.Epairs:
            self.Epairs.remove(box_name)

        self.calc_Vdir()
        self.calc_Vdim()

        found_vpair = self.find_v_pairs()
        for c in self.connections:
            neighbour = self.get_box_object(c)
            if neighbour:
                neighbour_found_vpair = neighbour.find_v_pairs()
                if neighbour_found_vpair:
                    found_vpair = True

        box_to_eat.merged_with(self.name)
        return found_vpair
