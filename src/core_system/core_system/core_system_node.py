import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from collections import deque

class CoreSystemNode(Node):
    def __init__(self):
        super().__init__('core_system_node')

        self.station_request_service = self.create_service(Trigger, 'station_request', self.station_request_callback)
        self.station_confirmation_service = self.create_service(Trigger, 'station_confirmation', self.station_confirmation_callback)
        self.manipulator_task_service = self.create_service(Trigger, 'manipulator_task', self.manipulator_task_callback)

        self.station_queue = deque()
        self.manipulator_task_ready = False  # True when all stations are completed

        self.get_logger().info("Core system node started, waiting for AMR requests...")

    def station_request_callback(self, request, response):
        self.get_logger().info("Received AMR station request.")

        while not self.station_queue:
            try:
                station_input = input("Enter multiple station numbers (e.g., 1,2,4) or '0' to stop: ")

                if station_input.strip() == '0':
                    self.get_logger().info("No stations assigned.")
                    response.success = False
                    return response

                station_list = [int(s) for s in station_input.split(",") if s.strip().isdigit()]
                valid_stations = [s for s in station_list if s in [1, 2, 3, 4]]

                if valid_stations:
                    self.station_queue.extend(valid_stations)
                    break
                else:
                    print("Invalid stations. Enter numbers between 1 and 4, separated by commas.")
            except ValueError:
                print("Invalid input. Enter numbers between 1 and 4, separated by commas.")

        self.get_logger().info(f"Assigned stations {list(self.station_queue)} to AMR.")
        response.success = True  
        return response

    def station_confirmation_callback(self, request, response):
        if self.station_queue:
            station = self.station_queue.popleft()
            self.get_logger().info(f"AMR confirmed arrival at Station {station}.")

            if not self.station_queue:  # All stations visited
                self.get_logger().info("All stations completed.")
                self.manipulator_task_ready = True

            response.success = True
        else:
            self.get_logger().info("No stations left in queue.")
            response.success = False

        return response

    def manipulator_task_callback(self, request, response):
        if self.manipulator_task_ready:
            self.get_logger().info("Starting manipulator.")
            response.success = True
            self.manipulator_task_ready = False  # Reset for next batch
        else:
            # self.get_logger().info("Waiting for all stations to be completed before starting task.")
            response.success = False  

        return response

def main(args=None):
    rclpy.init(args=args)
    core_system_node = CoreSystemNode()
    rclpy.spin(core_system_node)
    core_system_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
