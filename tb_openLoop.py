import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


'''Scenario 1:
The TurtleBot moved at a constant velocity to reach a specific coordinate 
along a straight line, following the equation x=v⋅t, 
where x represents the displacement, v is the constant velocity, and t 
is the time.'''
'''
class OpenLoopController(Node):
    def __init__(self): #initialises all the required parameters
        super().__init__('tb_openLoop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        self.duration = 10  # seconds
        self.velocity = 1  # m/s


   
    def timer_callback(self):
        elapsed_time = time.time() - self.start_time #less than 10 seconds as mentioned above
        if elapsed_time < self.duration:
            msg = Twist() #publish  Twist message to cml_vel topic
            msg.linear.x = self.velocity #publishes linear velocity in x direction to cmd_vel
            self.publisher_.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = 0.0 # stops the turtlebot as soon as 10 seconds have lapsed
            self.publisher_.publish(msg)
            self.timer.cancel() #stops the time


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopController() # Open Controller node is spun up
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


'''Scenario 2: 
The TurtleBot begins moving with an initial acceleration (a), accelerating 
until it reaches a certain speed (v). It then continues to move at this 
constant velocity (v) over a distance of x2​. Finally, the TurtleBot decelerates 
with a negative acceleration (- a) until it comes to a complete stop.
'''

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('tb_openloop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        
        # Define the phases
        self.acceleration_time = 2  # seconds
        self.constant_velocity_time = 3  # seconds
        self.deceleration_time = 2  # seconds
        self.total_time = self.acceleration_time + self.constant_velocity_time + self.deceleration_time
        
        # Define the velocities and accelerations
        self.acceleration = 0.5  # m/s^2
        self.velocity = float(self.acceleration * self.acceleration_time)  # m/s = a*t
        self.deceleration = float(self.velocity / self.deceleration_time)  # m/s^2 v/t

    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        msg = Twist()

        if elapsed_time < self.acceleration_time:#less than acceleration time
            msg.linear.x = float(self.acceleration * elapsed_time) #vx= a*t
        elif elapsed_time < self.acceleration_time + self.constant_velocity_time: #b/w acceleration time and constant velocity time
            msg.linear.x = float(self.velocity)#constant velocity
        elif elapsed_time < self.total_time: # if time is in final cycle of deceleration
            deceleration_time = elapsed_time - self.acceleration_time - self.constant_velocity_time
            msg.linear.x = float(self.velocity - self.deceleration * deceleration_time)#v=u-a*t
        else:
            msg.linear.x = 0.0 # after time elapses, velocity is 0
            self.timer.cancel()

        self.publisher_.publish(msg)#publish message to cmd_vel topic

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
