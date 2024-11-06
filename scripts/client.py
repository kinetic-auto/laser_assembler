#!/usr/bin/env python3

import time

import numpy as np

import rclpy
from laser_assembler_interfaces.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.node import Node


class LaserAssemblerClient(Node):
	def __init__(self, client_node_name='laser_assembler_client', server_node_name='point_cloud_assembler'):
		super().__init__(client_node_name)
		self.server_node_name = server_node_name

		self.get_logger().info(f'Connecting to server: {self.server_node_name}')

		service_name = self.server_node_name + '/assemble_scans2'
		self.assemble_cloud_client = self.create_client(AssembleScans2, service_name)
		while not self.assemble_cloud_client.wait_for_service(timeout_sec=1):
			self.get_logger().info(service_name + ' service not available, waiting...')

		self.scan_publisher = self.create_publisher(PointCloud2, '/scan', 1)

	def get_assembled_cloud(self, start_time, end_time):
		request = AssembleScans2.Request()
		request.begin = rclpy.time.Time(seconds=start_time).to_msg()
		request.end = rclpy.time.Time(seconds=end_time).to_msg()
		future = self.assemble_cloud_client.call_async(request)
		rclpy.spin_until_future_complete(self, future)
		return future.result()

def write_pointcloud2_to_xyz(cloud, filename):
	points = point_cloud2.read_points(cloud)
	validPoints = points[np.isfinite(points['x'])]
	np.savetxt(filename, validPoints, delimiter=',', fmt=['%.6f', '%.6f', '%.6f', '%d'])


def main(args=None):
	cur_time = time.time()

	rclpy.init(args=args)

	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument('server_node_name', help='Name of the laser_assembler node, typically \'point_cloud_assembler\'')
	parser.add_argument('--start_time', '-s', type=float, required=True,
		                help='Start time to build cloud, this is relative to the current time unless the --abs_time flag is set')
	parser.add_argument('--end_time', '-e', type=float,
		                help='End time to build cloud, this is relative to the current time unless the --abs_time flag is set. Defaults to 0')
	parser.add_argument('--abs_time', '-a', action='store_true',
		                help='Specifies that the start/end_time parameters are POSIX timestamps')
	parser.add_argument('--publish_result', '-p', action='store_true', help='Publish the resulting point cloud on /scan')
	args = parser.parse_args()

	client = LaserAssemblerClient(server_node_name=args.server_node_name)

	if not args.abs_time:
		start_time = cur_time - args.start_time
		if args.end_time is None:
			args.end_time = 0
		end_time = cur_time - args.end_time
	else:
		start_time = args.start_time
		if args.end_time is None:
			raise Exception('In --abs_time mode, --end_time must be set!')
		end_time = args.end_time

	print(f'Requesting assembled point cloud from {start_time:>.6f}')
	print(f'                                 to   {end_time:>.6f}')
	result = client.get_assembled_cloud(start_time, end_time)

	num_points = result.cloud.height * result.cloud.width
	print(f'Received point cloud with {num_points} points')

	if (args.publish_result):
		client.scan_publisher.publish(result.cloud)
		rclpy.spin(client)

	outputFilename = f'{args.server_node_name}_{start_time:.6f}_{end_time:.6f}.xyz'
	print(f'Writing point cloud to {outputFilename}')
	write_pointcloud2_to_xyz(result.cloud, outputFilename)

	client.destroy_node()
	rclpy.shutdown()
	print('Done.')


if __name__=='__main__':
	main()
