# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Trigger

# class AMRClient(Node):
#     def __init__(self):
#         super().__init__('amr_client')

#         # Initialize service clients for station request, station confirmation, and manipulator task
#         self.station_request_client = self.create_client(Trigger, 'station_request')
#         self.station_confirmation_client = self.create_client(Trigger, 'station_confirmation')
#         self.manipulator_task_client = self.create_client(Trigger, 'manipulator_task')

#         # Wait for services to be available
#         self._wait_for_service(self.station_request_client, 'station_request')
#         self._wait_for_service(self.station_confirmation_client, 'station_confirmation')
#         self._wait_for_service(self.manipulator_task_client, 'manipulator_task')

#     def _wait_for_service(self, client, service_name, timeout_sec=1.0):
#         """
#         Helper method to wait for a service to be available.
#         """
#         while not client.wait_for_service(timeout_sec=timeout_sec):
#             self.get_logger().info(f'Waiting for {service_name} service to be available...')

#     def request_station(self):
#         """
#         Sends a request for the station and waits for the response.
#         """
#         request = Trigger.Request()
#         future = self.station_request_client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)
#         response = future.result()

#         if response.success:
#             self.get_logger().info("Received station request confirmation from Core System.")
#             return True
#         else:
#             self.get_logger().info("Station request failed.")
#             return False

#     def confirm_station_reached(self):
#         """
#         Wait for user input to confirm that the AMR has reached the station.
#         """
#         user_input = input(f"Enter '1' to confirm AMR reached the station. (Station assigned by Core System)\n")
#         if user_input == '1':
#             self.get_logger().info("AMR reached the station.")
#             request = Trigger.Request()
#             future = self.station_confirmation_client.call_async(request)
#             rclpy.spin_until_future_complete(self, future)
#             response = future.result()

#             if response.success:
#                 self.get_logger().info("Station reach confirmed.")
#                 return True
#         else:
#             self.get_logger().info("Station reach not confirmed.")
#         return False

#     def trigger_manipulator_task(self):
#         """
#         Triggers the manipulator task after confirmation of station reach.
#         """
#         request = Trigger.Request()
#         future = self.manipulator_task_client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)
#         response = future.result()

#         if response.success:
#             self.get_logger().info("Manipulator task started successfully.")
#         else:
#             self.get_logger().info("Failed to start manipulator task.")

# def main(args=None):
#     rclpy.init(args=args)

#     amr_client = AMRClient()

#     # Request station from the core system
#     if amr_client.request_station():
#         # Wait for confirmation that AMR reached the station
#         if amr_client.confirm_station_reached():
#             # Trigger manipulator task
#             amr_client.trigger_manipulator_task()

#     amr_client.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class AMRClient(Node):
    def __init__(self):
        super().__init__('amr_client')

        self.station_request_client = self.create_client(Trigger, 'station_request')
        self.station_confirmation_client = self.create_client(Trigger, 'station_confirmation')
        self.manipulator_task_client = self.create_client(Trigger, 'manipulator_task')

        self._wait_for_service(self.station_request_client, 'station_request')
        self._wait_for_service(self.station_confirmation_client, 'station_confirmation')
        self._wait_for_service(self.manipulator_task_client, 'manipulator_task')

    def _wait_for_service(self, client, service_name, timeout_sec=1.0):
        while not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().info(f'Waiting for {service_name} service to be available...')

    def request_station(self):
        request = Trigger.Request()
        future = self.station_request_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            return True
        else:
            self.get_logger().info("No station assigned or process stopped.")
            return False

    def confirm_station_reached(self):
        while True:
            user_input = input("Enter '1' to confirm AMR reached the next station, or any other key to stop: ")
            if user_input == '1':
                self.get_logger().info("Confirming arrival at station...")
                request = Trigger.Request()
                future = self.station_confirmation_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()

                if response.success:
                    self.get_logger().info("Station reach confirmed.")

                    # Check if this was the last station
                    if not self.has_more_stations():
                        self.get_logger().info("All stations completed. Automatically starting manipulator task.")
                        self.trigger_manipulator_task()
                        return False  # Stop asking for confirmation after the last station
                else:
                    self.get_logger().info("No more stations to confirm.")
                    return False
            else:
                return False

        return True

    def has_more_stations(self):
        """
        Check with Core System if there are more stations left.
        If no more stations, return False.
        """
        request = Trigger.Request()
        future = self.station_request_client.call_async(request)  # Using station request service to check queue
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.success  # True if there are more stations, False if last station is reached

    def trigger_manipulator_task(self):
        self.get_logger().info("Requesting manipulator task after all stations are completed.")
        request = Trigger.Request()
        future = self.manipulator_task_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            self.get_logger().info("Manipulator task started successfully.")
        else:
            self.get_logger().info("Manipulator task not started.")

def main(args=None):
    rclpy.init(args=args)
    amr_client = AMRClient()

    if amr_client.request_station():
        while amr_client.confirm_station_reached():
            pass  # Confirm all stations one by one

        amr_client.trigger_manipulator_task()  # Start manipulator task once all stations are done

    amr_client.get_logger().info("All tasks completed. Shutting down AMR client.")
    amr_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
