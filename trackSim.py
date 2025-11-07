import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

class SE2(Enum):
    X = 0
    Y = 1
    O = 2
    V = 3
    A = 4
    W = 5

class Range(Enum):
    MIN = 0
    MAX = 1

class Index(Enum):
    ROW = 0
    COL = 1

class Simulator():
    """Tracking Simulator class."""
    def __init__(self, total_objects: int, total_datapoints: int):

        self.detection_prob = 0.5
        self.sample_rate = 0.1
        self.fov = (5, 10)
        self.speed = (20, 30)
        self.measurements = [set() for _ in range(total_datapoints)]
        self.objects = { 
            i: np.zeros((6, total_datapoints), dtype=np.float64) 
            for i in range(total_objects)}

        for array in self.objects.values():
            self.genTrajectory(array)

        self.populate_measurements()




    def genTrajectory(self, array : np.ndarray):
        array[SE2.Y.value, : ] = np.random.uniform(
            low = self.fov[Range.MIN.value], 
            high= self.fov[Range.MAX.value])

        array[SE2.O.value, : ] = np.random.uniform(
            low = -np.pi / 4, 
            high= np.pi / 4)

        array[SE2.Y.value, : ] = np.random.uniform(
            low = self.fov[Range.MIN.value], 
            high= self.fov[Range.MAX.value])

        array[SE2.V.value, : ] = np.random.uniform(
            low = self.speed[Range.MIN.value], 
            high = self.speed[Range.MAX.value])


        for k in range(1, array.shape[Index.COL.value]):
            # Constant velocity
            array[SE2.X.value, k] = array[SE2.X.value, k-1] + \
                self.sample_rate * array[SE2.V.value, k-1] * \
                np.cos(array[SE2.O.value, k-1])

            array[SE2.Y.value, k] = array[SE2.Y.value, k-1] + \
                self.sample_rate * array[SE2.V.value, k-1] * \
                np.sin(array[SE2.O.value, k-1])

            array[SE2.V.value, k] = array[SE2.V.value, k-1]

    def populate_measurements(self):
        # Add real measurements from objects
        for array in self.objects.values():
            for k in range(array.shape[Index.COL.value]):
                rand = np.random.uniform(0,1,1)
                if ((rand < self.detection_prob) or (k == 0)):
                    range_meas = np.sqrt(np.pow(array[SE2.X.value, k],2) +\
                        np.pow(array[SE2.Y.value, k],2))

                    bearing_meas = np.atan2(array[SE2.Y.value, k], array[SE2.X.value, k])

                    self.measurements[k].add((range_meas, bearing_meas))




if  __name__ == "__main__":
    sim = Simulator(3, 10)
    # Create the figure and axes
    fig, ax = plt.subplots(figsize=(10, 6))

    # Iterate through the dictionary to plot each object
    for key, data_array in sim.objects.items():
        # Basic check for a valid 2D array structure
        if data_array.ndim != 2 or data_array.shape[1] < 2:
            print(f"Skipping key {key}: Data array is not a valid 2D array with at least 2 columns.")
            continue

        # Extract coordinates:
        # x-coordinates are the first column (index 0)
        x_coords = data_array[0, : ]
        # y-coordinates are the second column (index 1)
        y_coords = data_array[1, : ]

        # Plot the data
        # 'label' is used by ax.legend() to create the legend entry for this plot
        # I use marker='o' to show the individual points clearly.
        ax.plot(x_coords, y_coords, label=f'Object {key}', marker='o')

    flat_data = []
    for set_of_measurements in sim.measurements:
        for measurement in set_of_measurements:
            flat_data.append(measurement)

    if not flat_data:
        raise ValueError("The flattened data list is empty. Cannot plot.")

    # 2. Separate into NumPy arrays
    ranges = np.array([r for r, b in flat_data])
    bearings_rad = np.array([b for r, b in flat_data]) # Bearings are already in radians

    # 3. Polar to Cartesian Conversion
    # x = range * cos(bearing)
    # y = range * sin(bearing)
    x_components = ranges * np.cos(bearings_rad)
    y_components = ranges * np.sin(bearings_rad)

    # 4. Plotting using plt.quiver

    # All vectors start at the origin (0, 0)
    origin_x = np.zeros_like(x_components)
    origin_y = np.zeros_like(y_components)

    # Use quiver to plot the vectors:
    # U, V are the x and y components (the final position)
    # The color is varied by vector index for visual distinction
    ax.quiver(
        origin_x, origin_y,            # X, Y: Starting coordinates (all 0, 0)
        x_components, y_components,    # U, V: Vector components (the calculated x and y)
        angles='xy', scale_units='xy', # Ensures vector length matches data units
        scale=1,                       # Scale=1 means 1 unit in data = 1 unit on screen
        width=0.0007,
        # linewidth=0.001,
        # headwidth=0.0, headlength=0.0,     # Adjust head size for better visibility
        label='Range & Bearing Vectors'
    )

    # Add labels, title, and legend
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Plot of Objects from Dictionary')
    ax.legend(title='Object ID') # Display the legend with the custom labels
    ax.grid(True)

    # To display or save the plot
    plt.show()
