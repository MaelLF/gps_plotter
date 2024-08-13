import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
import matplotlib.pyplot as plt

class GPSPlotter(Node):
    def __init__(self):
        super().__init__('gps_plotter')

        # Subscription for NavSatFix topic (/gnss)
        self.gnss_subscription = self.create_subscription(
            NavSatFix,
            '/gnss',  
            self.gnss_callback,
            10)
        self.gnss_subscription
        
        # Subscription for Vector3Stamped topic (/filter/positionlla)
        self.filter_subscription = self.create_subscription(
            Vector3Stamped,
            '/filter/positionlla',
            self.filter_callback,
            10)
        self.filter_subscription

        # Data storage
        self.gnss_latitudes = []
        self.gnss_longitudes = []
        self.filter_latitudes = []
        self.filter_longitudes = []

        # Plot setup
        plt.ion()
        self.figure, self.ax = plt.subplots()
        self.gnss_scatter, = self.ax.plot([], [], 'bo', label='GNSS')  # Blue for GNSS
        self.filter_scatter, = self.ax.plot([], [], 'ro', label='Filter')  # Red for Filter
        plt.title('GPS Positions')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.legend()

    def gnss_callback(self, msg):
        self.gnss_latitudes.append(msg.latitude)
        self.gnss_longitudes.append(msg.longitude)
        self.update_plot()

    def filter_callback(self, msg):
        self.filter_latitudes.append(msg.vector.x)  # Assuming vector.x is latitude
        self.filter_longitudes.append(msg.vector.y)  # Assuming vector.y is longitude
        self.update_plot()

    def update_plot(self):
        self.gnss_scatter.set_xdata(self.gnss_longitudes)
        self.gnss_scatter.set_ydata(self.gnss_latitudes)
        self.filter_scatter.set_xdata(self.filter_longitudes)
        self.filter_scatter.set_ydata(self.filter_latitudes)
        self.ax.relim()
        self.ax.autoscale_view()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    gps_plotter = GPSPlotter()
    rclpy.spin(gps_plotter)
    gps_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
