#!/usr/bin/env python3
"""
Battery Charging Action Server
================================
This simulates a robot charging station.
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from custom_interfaces.action import ChargeBattery


class BatteryCharger(Node):
    """
    The charging station that handles charging requests.
    """

    def __init__(self):
        super().__init__('battery_charger')
        
        # Starting battery level
        self.current_battery = 20  # Start at 20%
        
        # How fast we charge (5% per second)
        self.charge_rate = 5.0
        
        self.get_logger().info('=================================')
        self.get_logger().info('🔋 Battery Charging Station Ready')
        self.get_logger().info(f'   Current battery: {self.current_battery}%')
        self.get_logger().info(f'   Charge rate: {self.charge_rate}%/sec')
        self.get_logger().info('=================================')

        # Create the action server
        self.action_server = ActionServer(
            self,
            ChargeBattery,
            'charge_battery',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    # ------------------------------------------------------------
    # GOAL CALLBACK
    # ------------------------------------------------------------
    def goal_callback(self, goal_request):
        """
        Called when someone sends a charging goal.
        We accept or reject based on validity.
        """
        target = goal_request.target_percentage
        
        self.get_logger().info(f'📥 Charging request received: {target}%')
        
        # Check 1: Is target valid?
        if target < 0 or target > 100:
            self.get_logger().warn(f'❌ Invalid target: {target}%')
            return GoalResponse.REJECT
        
        # Check 2: Already charged enough?
        if self.current_battery >= target:
            self.get_logger().warn(
                f'❌ Already at {self.current_battery}% (target: {target}%)'
            )
            return GoalResponse.REJECT
        
        self.get_logger().info('✅ Goal accepted - Starting charge!')
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------
    # CANCEL CALLBACK
    # ------------------------------------------------------------
    def cancel_callback(self, goal_handle):
        """
        Called when the user cancels the charging.
        """
        self.get_logger().info('🛑 Cancel request received')
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------
    # EXECUTE CALLBACK
    # ------------------------------------------------------------
    def execute_callback(self, goal_handle):
        """
        Main loop that simulates battery charging.
        """
        self.get_logger().info('⚡ Charging started!')
        
        target = goal_handle.request.target_percentage
        start_time = time.time()
        
        feedback_msg = ChargeBattery.Feedback()
        
        # CHARGING LOOP
        while self.current_battery < target:
            
            # Handle cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                
                result = ChargeBattery.Result()
                result.success = False
                result.final_percentage = int(self.current_battery)
                result.charging_time = time.time() - start_time
                
                self.get_logger().info('🛑 Charging canceled!')
                return result
            
            # Charge a bit every 0.5 sec
            charge_amount = self.charge_rate * 0.5
            self.current_battery = min(self.current_battery + charge_amount, target)
            
            # Time remaining estimate
            remaining = target - self.current_battery
            time_remaining = remaining / self.charge_rate
            
            # Prepare feedback
            feedback_msg.current_percentage = int(self.current_battery)
            feedback_msg.time_remaining = time_remaining
            feedback_msg.charging_rate = self.charge_rate
            
            # Send feedback
            self.get_logger().info(
                f'🔋 Charging: {feedback_msg.current_percentage}% '
                f'(~{time_remaining:.1f}s left)'
            )
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.5)
        
        # CHARGING COMPLETE
        goal_handle.succeed()
        
        total_time = time.time() - start_time
        
        result = ChargeBattery.Result()
        result.success = True
        result.final_percentage = int(self.current_battery)
        result.charging_time = total_time
        
        self.get_logger().info('=================================')
        self.get_logger().info('✅ Charging complete!')
        self.get_logger().info(f'   Final: {result.final_percentage}%')
        self.get_logger().info(f'   Time: {result.charging_time:.1f}s')
        self.get_logger().info('=================================')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    charger = BatteryCharger()
    
    try:
        rclpy.spin(charger)
    except KeyboardInterrupt:
        charger.get_logger().info('Shutting down...')
    
    charger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
