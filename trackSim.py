import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

class SE2(Enum):
    """
    Defines the state space for the objects on a SE2 manifold.

    Attributes:
        X: inertial x coordinate
        Y: inertial y coordinate
        O: inertial heading/orientation
        V: velocity
        A: acceleration
        W: angular velocity
    """
    X = 0
    Y = 1
    O = 2
    V = 3
    A = 4
    W = 5

class Indicies(Enum):
    """
    Defines indicies for the field of view (FOV) and the measurements.

    Attributes:
        MIN: index of the minimium FOV.
        MAX: index of the maximum FOV.
        RANGE: index of range element.
        BEARING: index of the bearing element.
    """
    MIN = 0
    MAX = 1
    RANGE = 0
    BEARING = 1


class Simulator():
    """
    Generates tracks for point objects that can be used for single or multi object tracking algorithms. The measurements are in the form of range and bearing.

    Parameters:
        total_objects (int): total number point objects being simulated.
    Attributes:
        total_objects (int): total number point objects being simulated.
        datapoints (int): maximum number of object orginated data points.
        average_clutter (float): total number of clutter detections present for a given scan.
        sample_rate (float): period between samples.
        fov: (list): Field of view of the sensor (metres).
        speed (tuple): Minimum and maximum speed range of objects.
        measurements (list): Measurements potentially containing object detections 
        and clutter.
        noise (tuple): error variance for the range and bearing measurements.
        clutter (list): Measurements originated from clutter.
        object_detections (list): Object orginated measurements.
        objects (dict): Objects trajectories.
    """

    def __init__(self, total_objects: int):
        """
        Inititate new simulation instance. 
        """
        if (total_objects <= 0):
            raise ValueError("The total number of objects simulated must be \
                greater than zero.")

        self.total_objects : int = total_objects
        self.datapoints : int = 100
        self.average_clutter : float = 5.
        self.detection_prob : float = .9
        self.sample_rate : float = .01
        self.fov : list = [(0, 10),(-10, 10)]
        self.speed : tuple = (20, 30)
        self.noise : tuple = (.2, 0.01)

        self.measurements = [set() for _ in range(self.datapoints)]
        self.clutter = [set() for _ in range(self.datapoints)]
        self.object_detections = [set() for _ in range(self.datapoints)]

        self.objects = { 
            i: [[0] for _ in range(6)]
            for i in range(total_objects)}

        for array in self.objects.values():
            self.genTrajectory(array)

        self.populate_measurements()


    def genTrajectory(self, array : list):
        """
        Generates the trajectory of a given object in the inertial coordinate frame. 
        Populates the self.objects list with the state space resulting from the trajectory.

        Parameters:
            array: the list of the objects state vectors corresponding to 
            its trajectory.
        """
        array[SE2.Y.value][0] = np.random.uniform(
            low = self.fov[SE2.X.value][Indicies.MIN.value], 
            high= self.fov[SE2.X.value][Indicies.MAX.value])

        array[SE2.O.value][0] = np.random.uniform(
            low = -np.pi / 4, 
            high= np.pi / 4)

        array[SE2.V.value][0] = np.random.uniform(
            low = self.speed[Indicies.MIN.value], 
            high = self.speed[Indicies.MAX.value])


        for _ in range(1, self.datapoints):
            # Constant velocity
            array[SE2.X.value].append(array[SE2.X.value][-1] + \
                self.sample_rate * array[SE2.V.value][-1] * \
                np.cos(array[SE2.O.value][-1]))

            array[SE2.Y.value].append(array[SE2.Y.value][-1] + \
                self.sample_rate * array[SE2.V.value][-1] * \
                np.sin(array[SE2.O.value][-1]))

            array[SE2.V.value].append(array[SE2.V.value][-1])

            # Check if the object is outside the field of view
            if (array[SE2.X.value][-1] < self.fov[SE2.X.value][Indicies.MIN.value] or 
                    array[SE2.X.value][-1] > self.fov[SE2.X.value][Indicies.MAX.value]):
                break

            if (array[SE2.Y.value][-1] < self.fov[SE2.Y.value][Indicies.MIN.value] or 
                    array[SE2.Y.value][-1] > self.fov[SE2.Y.value][Indicies.MAX.value]):
                break


    def generate_clutter(self):
        """
        Generates clutter measurements uniformly within the sensors field of view.
        """
        for k in range(self.datapoints):
            total_clutter = np.random.poisson(self.average_clutter)
            for _ in range(total_clutter):
                clutter = np.random.uniform(low=[self.fov[0][0], self.fov[1][0]],
                                            high=[self.fov[0][1], self.fov[1][1]])

                range_meas = np.sqrt(np.pow(clutter[SE2.X.value],2) + \
                    np.pow(clutter[SE2.Y.value],2))

                bearing_meas = np.atan2(clutter[SE2.Y.value], 
                                        clutter[SE2.X.value])

                self.clutter[k].add((range_meas, bearing_meas))

    def generate_object_detections(self):
        """
        Uses the objects trajectory to generate range and bearing measurements. 
        Gaussian noise is then added subsequently.
        """
        for array in self.objects.values():

            for k in range(len(array[SE2.Y.value])):

                range_meas = np.sqrt(np.pow(array[SE2.X.value][k],2) +\
                    np.pow(array[SE2.Y.value][k],2)) +\
                    np.random.normal(0, self.noise[Indicies.RANGE.value])

                bearing_meas = np.atan2(array[SE2.Y.value][k], 
                                        array[SE2.X.value][k]) +\
                    np.random.normal(0, self.noise[Indicies.BEARING.value])


                self.object_detections[k].add((range_meas, bearing_meas))

    def populate_measurements(self):
        """
        Combines the clutter detections with the noisy object detections. 
        Additionally includes the probability of a missed detection.
        """
        self.generate_object_detections()
        self.generate_clutter()

        for k in range(len(self.measurements)):
            self.measurements[k] |= self.clutter[k]

            # Add missed detections
            rand = np.random.uniform(0,1,1)
            if ((rand < self.detection_prob) or (k == 0)):
                self.measurements[k] |= self.object_detections[k]

    def plot_trajectories(self, ax):
        """
        Plots the trajectory line that the object followed.
        Parameters:
            ax (Axis): matplotlib axis.
        """
        for key, data_array in self.objects.items():

            x_coords = data_array[SE2.X.value]
            y_coords = data_array[SE2.Y.value]

            ax.plot(x_coords, y_coords, label=f'Object {key}', marker='o')


    def plot_clutter(self, ax):
        """
        Plots the clutter detections as a scatter plot.
        Parameters:
            ax (Axis): matplotlib axis.
        """
        flat_data = []
        for set_of_measurements in sim.clutter:
            for measurement in set_of_measurements:
                flat_data.append(measurement)

        if not flat_data:
            raise ValueError("The flattened data list is empty. Cannot plot.")

        ranges = np.array([r for r, _ in flat_data])
        bearings_rad = np.array([b for _, b in flat_data]) 

        x_components = ranges * np.cos(bearings_rad)
        y_components = ranges * np.sin(bearings_rad)

        ax.scatter(x_components, y_components)


    def plot_detections(self, ax):
        """
        Plots the object detections as a vector from the origin to the detection.
        Parameters:
            ax (Axis): matplotlib axis.
        """
        flat_data = []
        for set_of_measurements in self.object_detections:
            for measurement in set_of_measurements:
                flat_data.append(measurement)

        if not flat_data:
            raise ValueError("The flattened data list is empty. Cannot plot.")

        ranges = np.array([r for r, _ in flat_data])
        bearings_rad = np.array([b for _, b in flat_data]) 

        x_components = ranges * np.cos(bearings_rad)
        y_components = ranges * np.sin(bearings_rad)

        origin_x = np.zeros_like(x_components)
        origin_y = np.zeros_like(y_components)

        ax.quiver(
            origin_x, origin_y,
            x_components, y_components,
            angles='xy',
            scale_units='xy', 
            scale=1,
            width=0.0007,
            label='Range & Bearing Vectors')

    def plot_measurements(self, ax):
        """
        Plots all measurements as a vector from the origin to the detection.
        Parameters:
            ax (Axis): matplotlib axis.
        """
        flat_data = []
        for set_of_measurements in self.measurements:
            for measurement in set_of_measurements:
                flat_data.append(measurement)

        if not flat_data:
            raise ValueError("The flattened data list is empty. Cannot plot.")

        ranges = np.array([r for r, _ in flat_data])
        bearings_rad = np.array([b for _, b in flat_data]) 

        x_components = ranges * np.cos(bearings_rad)
        y_components = ranges * np.sin(bearings_rad)

        origin_x = np.zeros_like(x_components)
        origin_y = np.zeros_like(y_components)

        ax.quiver(
            origin_x, origin_y,
            x_components, y_components,
            angles='xy',
            scale_units='xy', 
            scale=1,
            width=0.0007,
            label='Range & Bearing Vectors')


if  __name__ == "__main__":
    sim = Simulator(1)

    fig, ax = plt.subplots(figsize=(10, 6))

    sim.plot_trajectories(ax)

    sim.plot_detections(ax)
    # sim.plot_clutter(ax)

    ax.set_xlim(left=sim.fov[0][0], right=sim.fov[0][1])
    ax.set_ylim(bottom=sim.fov[1][0], top=sim.fov[1][1])
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Plot of Objects from Dictionary')
    ax.legend(title='Object ID')
    ax.grid(True)

    plt.show()
