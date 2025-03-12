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

        self.stations_remaining = 0  # Track number of stations left

    def _wait_for_service(self, client, service_name, timeout_sec=1.0):
        while not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().info(f'Waiting for {service_name} service to be available...')

    def request_station(self):
        request = Trigger.Request()
        future = self.station_request_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            self.stations_remaining = 4  # Assume 4 stations, adjust dynamically if needed
            self.get_logger().info(f"AMR assigned {self.stations_remaining} stations to complete.")
            return True
        else:
            self.get_logger().info("No station assigned or process stopped.")
            return False

    def confirm_station_reached(self):
        while self.stations_remaining > 0:
            user_input = input(f"Enter '1' to confirm AMR reached station {5 - self.stations_remaining}: ")
            if user_input == '1':
                self.get_logger().info(f"Confirming arrival at station {5 - self.stations_remaining}...")
                request = Trigger.Request()
                future = self.station_confirmation_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()

                if response.success:
                    self.get_logger().info(f"Station {5 - self.stations_remaining} reach confirmed.")
                    self.stations_remaining -= 1  # Reduce station count

                    if self.stations_remaining == 0:
                        return False  # Stop asking for confirmation after last station

                else:
                    self.get_logger().info("Failed to confirm station.")
                    return False
            else:
                return False
        return True

    def trigger_manipulator_task(self):
        self.get_logger().info("Requesting manipulator task.")
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
            pass  # Confirm stations in order

        amr_client.trigger_manipulator_task()  # Only trigger after all stations are confirmed

    amr_client.get_logger().info("All tasks completed. Shutting down AMR client.")
    amr_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
