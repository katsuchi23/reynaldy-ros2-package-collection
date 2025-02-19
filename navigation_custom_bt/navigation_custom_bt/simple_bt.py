#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import py_trees

# Define basic behavior classes for our robot manipulator task

class MoveToPreGrasp(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveToPreGrasp"):
        super(MoveToPreGrasp, self).__init__(name)
    
    def update(self):
        self.feedback_message = "Moving to pre-grasp position"
        self.logger.info("Moving to pre-grasp position")
        # print(self.feedback_message) can also print manually
        time.sleep(0.5)  # simulate action duration
        return py_trees.common.Status.SUCCESS

class GraspObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="GraspObject"):
        super(GraspObject, self).__init__(name)
    
    def update(self):
        self.feedback_message = "Grasping the object"
        self.logger.info(self.feedback_message)
        time.sleep(0.5)
        return py_trees.common.Status.SUCCESS

class MoveToPlace(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveToPlace"):
        super(MoveToPlace, self).__init__(name)
    
    def update(self):
        self.feedback_message = "Moving to place position"
        self.logger.info(self.feedback_message)
        # print(self.feedback_message)
        time.sleep(0.5)
        return py_trees.common.Status.SUCCESS

class ReleaseObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="ReleaseObject"):
        super(ReleaseObject, self).__init__(name)
    
    def update(self):
        self.feedback_message = "Releasing the object"
        self.logger.info(self.feedback_message)
        # print(self.feedback_message)
        time.sleep(0.5)
        return py_trees.common.Status.SUCCESS

# Now, we create a ROS2 node that ticks our behavior tree

class ManipulatorBTNode(Node):
    def __init__(self):
        super().__init__('manipulator_bt_node')
        self.get_logger().info("Starting Manipulator BT Node")
        # Create the behavior tree
        self.tree = self.create_bt()
        self.bt = py_trees.trees.BehaviourTree(root=self.tree)
        self.bt.setup(timeout=15) # maximum time in sec for all nodes to finish their setup. If more than this will return error

        # tick only one
        # self.tick_tree()

        # Create a timer to tick the tree every certain time -> can tick multiple times
        self.tick_tree() # start with tick 1 times first
        self.timer = self.create_timer(5, self.tick_tree)

        # When the bt is ticked, it started from the root

    def create_bt(self):
        # Use a Sequence to run actions in order
        root = py_trees.composites.Sequence("ManipulatorSequence", memory=True)
        root.add_children([
            MoveToPreGrasp(),
            GraspObject(),
            MoveToPlace(),
            ReleaseObject()
        ])
        return root

    def tick_tree(self):
        self._logger.info("BT has begun")

        self.bt.tick() # so this code run during the whole process not just when the root is ticked
        # Optionally, print an ascii representation of the tree
        tree_ascii = py_trees.display.unicode_tree(root=self.tree)
        self.get_logger().info("\n" + tree_ascii)

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
