import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class MyPublisher(Node):

	def __init__(self):
		super().__init__('my_publisher')

		self.subscriber_battery = self.create_subscription(Float32, 'battery_topic', self.battery_callback, 5)
		self.subscriber_memory = self.create_subscription(Float32, 'available_memory', self.memory_callback, 5)
		self.subscriber_cpu = self.create_subscription(Float32, 'cpu_utilization', self.cpu_callback, 5)
		self.publisher = self.create_publisher(DiagnosticArray, 'diagnostics', 5)

		self.declare_parameter('safe_voltage_range_max', 1.0)
		self.declare_parameter('safe_voltage_range_min', 0.1)
		self.declare_parameter('safe_capacity_range_max', 1.0)
		self.declare_parameter('safe_capacity_range_min', 0.1)
		self.declare_parameter('safe_utilization_range_max', 0.8)
		self.declare_parameter('safe_utilization_range_min', 0.0)
	
	def battery_callback(self, msg):
		voltage = msg.data

		min_voltage = self.get_parameter('safe_voltage_range_min').get_parameter_value().double_value
		max_voltage = self.get_parameter('safe_voltage_range_max').get_parameter_value().double_value

		print(min_voltage)
		if not self.is_voltage_safe(voltage, min_voltage, max_voltage):
			self.publish_warning_battery(voltage)

	def is_voltage_safe(self, voltage, min_voltage, max_voltage):
		return min_voltage <= voltage <= max_voltage
	
	def publish_warning_battery(self, voltage):
		diagnostic_array_msg = DiagnosticArray()
		diagnostic_status_msg = DiagnosticStatus()
		diagnostic_status_msg.name = 'Battery Warning'
		diagnostic_status_msg.level = DiagnosticStatus.WARN
		diagnostic_status_msg.message = 'Low Battery Voltage!'
		diagnostic_status_msg.values = [KeyValue(key='Voltage', value=str(voltage))]
		diagnostic_array_msg.status.append(diagnostic_status_msg)
		self.publisher.publish(diagnostic_array_msg)
        
	def memory_callback(self, msg):
		capacity = msg.data

		min_capacity = self.get_parameter('safe_capacity_range_min').get_parameter_value().double_value
		max_capacity = self.get_parameter('safe_capacity_range_max').get_parameter_value().double_value
		
		if not self.is_capacity_safe(capacity, min_capacity, max_capacity):
			self.publish_warning_memory(capacity)
    	
	def is_capacity_safe(self, capacity, min_capacity, max_capacity):
		return min_capacity < capacity <= max_capacity
	
	def publish_warning_memory(self, capacity):
		diagnostic_array_msg = DiagnosticArray()
		diagnostic_status_msg = DiagnosticStatus()
		diagnostic_status_msg.name = 'Working Memory Warning'
		diagnostic_status_msg.level = DiagnosticStatus.WARN
		diagnostic_status_msg.message = 'Low Capacity of Working Memory!'
		diagnostic_status_msg.values = [KeyValue(key='Capacity', value=str(capacity))]
		diagnostic_array_msg.status.append(diagnostic_status_msg)
		self.publisher.publish(diagnostic_array_msg)

	def cpu_callback(self, msg):
		utilization = msg.data

		min_utilization = self.get_parameter('safe_utilization_range_min').get_parameter_value().double_value
		max_utilization = self.get_parameter('safe_utilization_range_max').get_parameter_value().double_value

		if not self.is_utilization_safe(utilization, min_utilization, max_utilization):
			self.publish_warning_cpu(utilization)
    	
	def is_utilization_safe(self, utilization, min_utilization, max_utilization):
		return min_utilization <= utilization <= max_utilization
	
	def publish_warning_cpu(self, utilization):
		diagnostic_array_msg = DiagnosticArray()
		diagnostic_status_msg = DiagnosticStatus()
		diagnostic_status_msg.name = 'CPU Warning'
		diagnostic_status_msg.level = DiagnosticStatus.WARN
		diagnostic_status_msg.message = 'High CPU Utilization!'
		diagnostic_status_msg.values = [KeyValue(key='utilization', value=str(utilization))]
		diagnostic_array_msg.status.append(diagnostic_status_msg)
		self.publisher.publish(diagnostic_array_msg)


def main(args=None):
	rclpy.init(args=args)
	
	analyzer = MyPublisher()

	rclpy.spin(analyzer)
	analyzer.destroy_node()
	rclpy.shutdown()
		


if __name__ == '__main__':
	main()
