import rclpy
from rclpy.node import Node

from rhover_control_interfaces.msg import EngineState, ErrorStatus
from rhover_control_interfaces.srv import AdjustSpeed, BatteryState, GetEngineState, Stop


class EngineServer(Node):
    """
    Create a new engine server instance.
    """
    def __init__(self):
        super().__init__('rhover_engine_server')
        self.get_logger().info('rhover_control.engine_server.start')

        self._state = EngineState()
        self._state.on = False
        self._state.speed = 0.0
        self._state.battery = 100

        self.engine_state_srv = self.create_service(GetEngineState, 'get_engine_state', self.handle_get_engine_state)
        self.adjust_speed_srv = self.create_service(AdjustSpeed, 'adjust_speed', self.handle_adjust_speed)
        self.stop_srv = self.create_service(Stop, 'stop', self.handle_stop)
        self.battery_state_srv = self.create_service(BatteryState, 'battery_state', self.handle_battery_state)


    def handle_get_engine_state(self, request, response):
        """
        Server method to return the current state of the engine.
        """
        response.state = self._state

        return response

    def handle_adjust_speed(self, request, response):
        """
        Method to change the Rhover's speed.

        Negative values for speed can be used to slow it down.
        """
        if self._state.speed == 0 and request.speed < 0:
            response.err = ErrorStatus()
            response.err.err = True
            response.err.errcode = 1
            response.err.errmsg = 'rhover is stopped'
            return response

        speed = self._state.speed + request.speed
        if speed < 0:
            speed = 0.0
        self._state.speed = speed

        if not self._state.on:
            self._state.on = True

        battery = self._state.battery - 10
        if battery <= 0:
            self._state.battery = 0
            self._state.speed = 0.0
            self._state.on = False
            response.err = ErrorStatus()
            response.err.err = True
            response.err.errcode = 1
            response.err.errmsg = 'rhover is out of energy'
            return response
        
        self._state.battery = battery

        response.err = ErrorStatus()
        response.err.err = False
        
        return response
    
    def handle_stop(self, request, response):
        """
        Stops the Rhover, reducing its speed to 0.
        """
        response.err = ErrorStatus()
        
        if self._state.speed == 0.0:
            response.err.err = True
            response.err.errcode = 1
            response.err.errmsg = 'rhover is already stopped'
            return response

        self._state.speed = 0.0
        response.err.err = False
        
        return response

    def handle_battery_state(self, request, response):
        """
        Return/inform the battery state.
        """
        if self._state.battery > 33 and self._state.battery <= 66:
            response.ok = False
            response.warn = True
            response.danger = False
        elif self._state.battery <= 33:
            response.ok = False
            response.warn = False
            response.danger = True
        else:
            response.ok = True
            response.warn = False
            response.danger = False

        return response


def main(args=None):
    rclpy.init(args=args)

    node = EngineServer()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
