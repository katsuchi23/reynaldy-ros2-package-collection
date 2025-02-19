import time
import rclpy
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import Twist
import math
from turtlesim.msg import Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveForward", node=None):
        super(MoveForward, self).__init__(name)
        self.node = node  # Fixed: Changed from Node to node
        self.publisher = None
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

    def setup(self, **kwargs):
        self.publisher = self.node.create_publisher(
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=self.qos_profile
        )        
        return True  # Fixed: Changed from super().setup(**kwargs)
    
    def update(self):
        msg = Twist()
        msg.linear.x = 1.0
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Publish cmd_vel: linear.x = {msg.linear.x}")
        return py_trees.common.Status.SUCCESS
    
class TwistNode(py_trees.behaviour.Behaviour):  # Fixed: Renamed class to avoid conflict
    def __init__(self, name="TwistNode", node=None):
        super(TwistNode, self).__init__(name)
        self.node = node  # Fixed: Changed from Node to node
        self.publisher = None
        self.duration = math.pi / 2
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

    def setup(self, **kwargs):
        self.publisher = self.node.create_publisher(  # Fixed: Changed from subscriber to publisher
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=self.qos_profile
        )
        return True
    
    def update(self):
        msg = Twist()
        msg.angular.z = 1.0
        end_time = self.node.get_clock().now().seconds_nanoseconds()[0] + self.duration

        while self.node.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.publisher.publish(msg)  # Fixed: Changed from publisher_ to publisher


        self.node.get_logger().info(f"Publish cmd_vel: angular.z = {msg.angular.z}")
        return py_trees.common.Status.SUCCESS
    
class IsNearWall(py_trees.behaviour.Behaviour):
    def __init__(self, name="IsNearWall", node=None):
        super(IsNearWall, self).__init__(name)
        self.node = node
        self.pose = None
        self.subscriber = None
        self.wall_threshold = 0.5
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

    def setup(self, **kwargs):
        self.subscriber = self.node.create_subscription(
            msg_type=Pose,
            topic='/turtle1/pose',
            callback=self.pose_callback,
            qos_profile=self.qos_profile
        )
        return True

    def pose_callback(self, msg):
        self.pose = msg

    def update(self):
        if self.pose is None:
            self.node.get_logger().warn("Pose data not received yet.")
            return py_trees.common.Status.FAILURE

            # we want to check if turtle move forward along 1 m it hit wall or not, hence we also need the theta of the pose
            
        x_final = self.pose.x + math.cos(self.pose.theta) # already input angle in radian
        y_final = self.pose.y + math.sin(self.pose.theta)
        print(self.pose.theta, x_final, y_final)

        if (x_final <= self.wall_threshold or 
            x_final >= 11 - self.wall_threshold or
            y_final <= self.wall_threshold or 
            y_final >= 11 - self.wall_threshold):
            self.node.get_logger().info("Turtle is near the wall.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info("Turtle is not near the wall.")
            return py_trees.common.Status.FAILURE

class TurtlelismBTNode(Node):
    def __init__(self):
        super().__init__('Turtlelism_bt_node')
        self.get_logger().info("Starting Turtlelism BT Node")
        self.count = 0  # Fixed: Added count attribute

        self.tree = self.create_bt()
        self.bt = py_trees.trees.BehaviourTree(root=self.tree)
        self.bt.setup(timeout=15)

        self.tick_tree()
        self.timer = self.create_timer(3, self.tick_tree) # make it faster

    def create_bt(self):
        root = py_trees.composites.Selector("RootSelector", memory=False)
        wall_sequence = py_trees.composites.Sequence("WallSequence", memory=False)
        
        wall_sequence.add_children([
            IsNearWall(node=self),
            TwistNode(node=self),  # Fixed: Changed Twist to TwistNode
            MoveForward(node=self) # if near wall, twist and also move forward and not just twist
        ])

        root.add_children([
            wall_sequence,
            MoveForward(node=self)
        ])
        return root

    def tick_tree(self):
        self.bt.tick()

        self.count += 1
        if self.count % 50 == 0:
            tree_ascii = py_trees.display.unicode_tree(self.tree)
            self.get_logger().info("\n" + tree_ascii)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlelismBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()