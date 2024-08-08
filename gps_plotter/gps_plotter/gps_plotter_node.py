import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt

class GPSPlotter(Node):
    def __init__(self):
        super().__init__('gps_plotter')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gnss',  
            self.listener_callback,
            10)
        self.subscription  
        self.latitudes = []
        self.longitudes = []
        plt.ion()
        self.figure, self.ax = plt.subplots()
        self.scatter, = self.ax.plot([], [], 'bo')
        plt.title('GPS Positions')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        self.x = 0

    def listener_callback(self, msg):
        if (self.x < 100):
            self.latitudes.append(msg.latitude)
            self.longitudes.append(msg.longitude)
            self.update_plot()
        else :
            print('stopped')

    def update_plot(self):
        self.scatter.set_xdata(self.longitudes)
        self.scatter.set_ydata(self.latitudes)
        self.ax.relim()
        self.ax.autoscale_view()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        self.x = self.x+1

def main(args=None):
    rclpy.init(args=args)
    gps_plotter = GPSPlotter()
    rclpy.spin(gps_plotter)
    gps_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
