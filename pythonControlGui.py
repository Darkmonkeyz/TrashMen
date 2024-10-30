import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from tkinter import Tk, Button

class GUI(Node):
    def __init__(self):
        super().__init__('gui_node')

        # Initialize service clients
        self.diversion_client = self.create_client(Trigger, 'request_diversion')
        self.e_stop_client = self.create_client(Trigger, 'e_stop_service')  # Change to your e-stop service name
        

        # Wait for the services to become available
        while not self.diversion_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /request_diversion service...')
        while not self.e_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /e_stop_service...')
        

        # Create the GUI
        self.root = Tk()
        self.root.title("Robot Control GUI")

        # Create buttons
        self.diversion_button = Button(self.root, text="Trigger Diversion", command=self.trigger_diversion)
        self.diversion_button.pack(pady=10)

        self.e_stop_button = Button(self.root, text="Emergency Stop", command=self.trigger_e_stop)
        self.e_stop_button.pack(pady=10)

        self.pause_nav_button = Button(self.root, text="Pause Navigation", command=self.trigger_pause_nav)
        self.pause_nav_button.pack(pady=10)

    def trigger_diversion(self):
        request = Trigger.Request()
        future = self.diversion_client.call_async(request)
        future.add_done_callback(self.diversion_response)

    def diversion_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Diversion triggered: ' + response.message)
            else:
                self.get_logger().warn('Diversion failed: ' + response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def trigger_e_stop(self):
        request = Trigger.Request()
        future = self.e_stop_client.call_async(request)
        future.add_done_callback(self.e_stop_response)

    def e_stop_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Emergency stop triggered: ' + response.message)
            else:
                self.get_logger().warn('E-stop failed: ' + response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def trigger_pause_nav(self):
        request = Trigger.Request()
        future = self.pause_nav_client.call_async(request)
        future.add_done_callback(self.pause_nav_response)

    def pause_nav_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Navigation paused: ' + response.message)
            else:
                self.get_logger().warn('Pause navigation failed: ' + response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui = GUI()
    gui.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
