# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Trigger

# class CoreSystemNode(Node):
#     def __init__(self):
#         super().__init__('core_system_node')

#         # Create service servers for 'station_request', 'station_confirmation', and 'manipulator_task'
#         self.station_request_service = self.create_service(Trigger, 'station_request', self.station_request_callback)
#         self.station_confirmation_service = self.create_service(Trigger, 'station_confirmation', self.station_confirmation_callback)
#         self.manipulator_task_service = self.create_service(Trigger, 'manipulator_task', self.manipulator_task_callback)

#         self.station_number = None  # Station number to send to AMR
#         self.manipulator_task_flag = False  # Flag to determine if manipulator task should start

#         self.get_logger().info("Core system node started, waiting for AMR request...")

#     def station_request_callback(self, request, response):
#         """
#         Callback for handling station requests from AMR.
#         Respond with a station number and wait for confirmation.
#         """
#         self.get_logger().info("Received AMR station request.")
        
#         # Ask user for station input
#         while self.station_number not in [1, 2, 3, 4]:
#             try:
#                 self.station_number = int(input("Please enter a station number (1-4): "))
#                 if self.station_number not in [1, 2, 3, 4]:
#                     print("Invalid station. Please enter a number between 1 and 4.")
#             except ValueError:
#                 print("Invalid input. Please enter a valid integer (1-4).")

#         self.get_logger().info(f"Sending station {self.station_number} to AMR.")
#         response.success = True  # Respond with station request success
#         return response

#     def station_confirmation_callback(self, request, response):
#         """
#         Callback for receiving confirmation from AMR that the station was reached.
#         Trigger manipulator task when confirmation is received.
#         """
#         self.get_logger().info("Received station confirmation from AMR.")
        
#         # Start manipulator task after station is confirmed
#         self.manipulator_task_flag = True
#         self.get_logger().info("Manipulator task flag set to True.")
        
#         response.success = True  # Confirm the station reach
#         return response

#     def manipulator_task_callback(self, request, response):
#         """
#         Callback to trigger manipulator task when the station confirmation flag is set.
#         """
#         if self.manipulator_task_flag:
#             self.get_logger().info("Starting manipulator task.")
#             response.success = True
#             self.manipulator_task_flag = False  # Reset the flag after task starts
#             self.station_number = None  # Reset the station number to wait for next request
#             self.get_logger().info("Manipulator task completed. Ready for next request.")
#         else:
#             self.get_logger().info("Waiting for station confirmation before starting task.")
#             response.success = False  # Task doesn't start until confirmation
#         return response

# def main(args=None):
#     rclpy.init(args=args)

#     core_system_node = CoreSystemNode()

#     # Wait for AMR to request station
#     rclpy.spin(core_system_node)

#     core_system_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

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
                self.get_logger().info("All stations completed. Preparing manipulator task.")
                self.manipulator_task_ready = True

            response.success = True
        else:
            self.get_logger().info("No stations left in queue.")
            response.success = False

        return response

    def manipulator_task_callback(self, request, response):
        if self.manipulator_task_ready:
            self.get_logger().info("Starting manipulator task after all stations completed.")
            response.success = True
            self.manipulator_task_ready = False  # Reset for next batch
        else:
            self.get_logger().info("Waiting for all stations to be completed before starting task.")
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
