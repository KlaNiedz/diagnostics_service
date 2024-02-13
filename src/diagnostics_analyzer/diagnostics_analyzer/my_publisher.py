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
		self.safe_voltage_range = (0.1, 1.0)
		self.safe_capacity_range = (0.15, 1.0)
		self.safe_utilization_range = (0, 0.8)

	def battery_callback(self, msg):
		voltage = msg.data
		voltage = round(voltage, 1)

		if not self.is_voltage_safe(voltage):
			self.publish_warning_battery(voltage)

	def is_voltage_safe(self, voltage):
		return self.safe_voltage_range[0] < voltage <= self.safe_voltage_range[1]
        
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
		capacity = round(capacity, 2)
		
		if not self.is_capacity_safe(capacity):
			self.publish_warning_memory(capacity)
    	
	def is_capacity_safe(self, capacity):
		return self.safe_capacity_range[0] < capacity <= self.safe_capacity_range[1]
	
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
		utilization = round(utilization, 2)
		
		if not self.is_utilization_safe(utilization):
			self.publish_warning_memory(utilization)
    	
	def is_utilization_safe(self, utilization):
		return self.safe_utilization_range[0] <= utilization <= self.safe_utilization_range[1]
	
	def publish_warning_memory(self, utilization):
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
