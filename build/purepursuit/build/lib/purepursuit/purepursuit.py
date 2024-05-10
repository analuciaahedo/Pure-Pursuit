import rclpy
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class pursuit_node(Node):
 def __init__(self):
    super().__init__('purepursuit')
    self.p_x=0.0
    self.p_y=0.0
    self.p_a=0.0
   
    self.linearV=0.11
    self.lookahead=1.0
    self.cmd_vel=Twist()

    self.velpsx_subscription=self.create_subscription(Float32, 'PosicionX',self.PosX_callback,rclpy.qos.qos_profile_sensor_data) 
    self.velpsx_subscription=self.create_subscription(Float32, 'PosicionY',self.PosY_callback,rclpy.qos.qos_profile_sensor_data) 
    self.theta_subscription=self.create_subscription(Float32, 'Orientacion',self.PosA_callback,rclpy.qos.qos_profile_sensor_data) 
    self.pub_cmdV=self.create_publisher(Twist,'cmd_vel',rclpy.qos.qos_profile_sensor_data) 

    
    self.timer_period=0.01
    self.create_timer(self.timer_period,self.pure_timer_callback) 
    self.get_logger().info('Algoritmo Pure Pursuit inicializado\n') 



    # CUADRADO
    self.paths = [
    [5, 10],
    [6, 9],
    [7, 8],
    [8, 7],
    [9, 6],
    [10, 6],
    [11, 6],
    [12, 6],
    [13, 6],
    [14, 6],
    [15, 6],
    [16, 6],
    [17, 6],
    [18, 6],
    [19, 5],
    [20, 5],
    [21, 5],
    [22, 5],
    [23, 5],
    [24, 5],
    [25, 5],
    [26, 5],
    [27, 5],
    [28, 5],
    [29, 5],
    [30, 5],
    [31, 5],
    [32, 5],
    [33, 5],
    [34, 5],
    [35, 5],
    [36, 5],
    [37, 5],
    [38, 5],
    [39, 5],
    [40, 5],
    [41, 5],
    [42, 5],
    [43, 5],
    [44, 5],
    [45, 5]
]
    
    self.get_logger().info('Pure pursuit node runnung !!!') 
    
 def PosX_callback(self,msg):
    self.p_x=msg.data 


 def PosY_callback(self,msg):
    self.p_y=msg.data 

 def PosA_callback(self,msg):
    self.p_a=msg.data 
 
 def cercania(self):
       dis_cerca = float('inf')
       punto_cerca = None
       for point in self.paths:
           distance = np.sqrt((self.p_x - point[0])**2 + (self.p_y - point[1])**2)
           if distance < dis_cerca:
               dis_cerca = distance
               punto_cerca = point
       if punto_cerca is None:
           return None
       next_index = self.paths.index(punto_cerca) + 1
       if next_index < len(self.paths):
           for point in self.paths[next_index:]:
               if np.sqrt((self.p_x - point[0])**2 + (self.p_y - point[1])**2) > self.lookahead:
                   return point
       return punto_cerca  # Return the closest point if no further point satisfies the condition

 def distance(self, p1, p2):
       return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

 def angle_to_target(self, target_point):
       dt_x = target_point[0] - self.p_x
       dt_y = target_point[1] - self.p_y

       angle_to_target = np.arctan2(dt_y, dt_x)
       alpha = angle_to_target - self.p_a
       k = 2 * np.sin(alpha) / self.lookahead
       return k
 



 # Example usage
 def pure_timer_callback(self):
     target_point = self.cercania()
     if target_point is not None:
        k = self.angle_to_target(target_point)
        self.get_logger().info(f'Pos en X: {self.p_x:.2f} Pos en Y: {self.p_y:.2f} Angulo: {self.p_a:.2f}')

        arrival_point = np.sqrt((self.p_x - target_point[0])**2 + (self.p_y - target_point[1])**2)
        if arrival_point < self.lookahead:  
            self.get_logger().info('Reached final target point \n')
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
        else:
            self.cmd_vel.linear.x = self.linearV
            self.cmd_vel.angular.z = k * self.linearV
        self.pub_cmdV.publish(self.cmd_vel)
     else:
        self.get_logger().info('No target point found, stopping robot.')
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.pub_cmdV.publish(self.cmd_vel)

def main(args=None):
   rclpy.init(args=args)


def main(args=None):
   rclpy.init(args=args)
   process = pursuit_node()
   rclpy.spin(process)
   process.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()


