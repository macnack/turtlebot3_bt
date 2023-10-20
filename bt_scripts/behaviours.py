import typing
import time
import py_trees
from py_trees import common
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import rclpy.qos
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

class Rotate(py_trees.behaviour.Behaviour):
	def __init__(self, 
				 name: str,
				 topic_name: str="/cmd_vel",
				 counterclockwise: bool=False,
				 vel: float=0.2):
		super(Rotate, self).__init__(name=name)
		self.topic_name = topic_name
		self.counterclockwise = counterclockwise
		self.vel = vel

	def setup(self, **kwargs):
		self.logger.debug("{}.setup()".format(self.qualified_name))
		try:
			self.node = kwargs['node']
		except KeyError as e:
				error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
				raise KeyError(error_message) from e  # 'direct cause' traceability
		
		self.publisher = self.node.create_publisher(
            msg_type=geometry_msgs.Twist,
            topic=self.topic_name,
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
		)
		self.feedback_message = "publisher created"

	def update(self) -> py_trees.common.Status:
		self.logger.debug("%s.update()" % self.__class__.__name__)
		msg = geometry_msgs.Twist()
		msg.angular.z = self.vel * ((-1)** (1*(not self.counterclockwise)))
		self.publisher.publish(msg)
		self.feedback_message = "Publishing rotation:" + str(msg.angular.z)
		return py_trees.common.Status.SUCCESS
	
	def terminate(self, new_status: py_trees.common.Status) -> None:
		self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
		)