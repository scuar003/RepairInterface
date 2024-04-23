import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs

import open3d as o3d

g_voxel_size = 0.01 # downsample pointcloud

def vector(x,y,z):
   return np.array([x,y,z])   

def vector2pose(point):
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]
    return pose

def numpy_from_pc2(points_pc2):
    # Convert PointCloud2 to numpy array
    gen = point_cloud2.read_points(points_pc2, skip_nans=True, field_names=("x", "y", "z")) # using a Python gen, get the xyz coords
    points_np = np.array(list(gen)) #convert gen values to list to numpy array
    return points_np
    
def tuples_to_matrix(old):
    N = len(old)  # Determine the number of elements in old
    new = np.zeros((N, 3))  # Initialize an empty 2D array of shape (N, 3)
    # Iterate over the structured array and explicitly extract x, y, z values
    for i in range(N):
        new[i, 0] = old[i]['x']
        new[i, 1] = old[i]['y']
        new[i, 2] = old[i]['z']
    return new
    
def o3d_from_numpy(points_np):
    points_o3d = o3d.geometry.PointCloud() #create empty o3d object
    points_o3d.points = o3d.utility.Vector3dVector(points_np) #populate o3d object with points
    return points_o3d

def msg_to_open3d(msg):
    points_np = numpy_from_pc2(msg)
    points_np = tuples_to_matrix(points_np)
    points_o3d = o3d_from_numpy(points_np)
    return points_o3d

def remove_outliers(o3d_pointcloud, num_of_neighbors, standard_ratio): #lower std_ratio = aggressive filtering, ie keep points a couple std dev away from mean of nb_neigh
    #Increasing the nb_neighbors parameter affects the filtering process in statistical
    #outlier removal by expanding the local neighborhood around each point. 
    cleaned_o3d_pointcloud, indices_of_cleaned_points = o3d_pointcloud.remove_statistical_outlier(nb_neighbors=num_of_neighbors,std_ratio=standard_ratio)
    return cleaned_o3d_pointcloud

def detect_planes(pointcloud): # List of all planes, as obox objects, given od3 pointcloud
    points = pointcloud.voxel_down_sample(voxel_size = g_voxel_size) 
    points.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)) # Calcluate normals for each point
    assert (points.has_normals()) #check if normals exist, else rise error
    # detect planar patches
    planes = points.detect_planar_patches(
            normal_variance_threshold_deg = 30,
            coplanarity_deg = 85,
            outlier_ratio = 1,
            min_plane_edge_length = 0.05,
            min_num_points = 0,
            search_param = o3d.geometry.KDTreeSearchParamKNN(knn=30))
    return planes

def get_center_of_plane_of_interest(plane_of_interest): #get center of a single plane
    center = plane_of_interest.get_center()
    return center

def get_corners_of_plane_of_interest(plane_of_interest): #get the list of corners of a single plane, each element in the list is the corner as a np array
    center_of_plane_of_interest = get_center_of_plane_of_interest(plane_of_interest)
    R = plane_of_interest.R
    extents = plane_of_interest.extent
    # The plane's normal is assumed to be along the axis with the smallest extent
    min_extent_idx = np.argmin(extents)
    indices = [0, 1, 2]
    indices.remove(min_extent_idx)
    # Compute the four corners of the plane
    corners_of_plane_of_interest = []
    for i in [-1, 1]:
        for j in [-1, 1]:
            corner_vect = np.zeros(3)
            corner_vect[indices[0]] = extents[indices[0]] / 2 * i
            corner_vect[indices[1]] = extents[indices[1]] / 2 * j
            # Ensure the corners are on the plane by making the component along the normal very small or zero
            corner_vect[min_extent_idx] = 0  # Place the corner on the plane, assuming the plane is at the center
            corner = center_of_plane_of_interest + R.dot(corner_vect)
            corners_of_plane_of_interest.append(corner)
    # Convert each NumPy array corner to a list
    corners_of_plane_of_interest = [corner.tolist() for corner in corners_of_plane_of_interest]
    return corners_of_plane_of_interest

def omit_far_points(max_distance, pcd):
    cam_pos = np.array([0,0,0]) #camera position
    
    points = np.asarray(pcd.points)
    
    #Compute squared distances to avoid sqrt computation for performance
    squared_distances = np.sum((points - cam_pos)**2, axis = 1)
    mask = squared_distances <= max_distance**2
    # Filter points using the mask
    filtered_points = points[mask]
    # Create a new point cloud with the filtered points
    filtered_pcd = o3d.geometry.PointCloud() # create empty open3d pointcloud
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points) # add points
    return filtered_pcd

class PlaneFinder(Node):
    def __init__(self):
        super().__init__('surface_detection')
        self.subscription = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.camera_callback, 1)
        self.publisher_planes = self.create_publisher(PoseArray, 'detected_surfaces', 10)
        self.command_subscriber = self.create_subscription(String, 'menu_action', self.command_callback, 1)
        # Transofrms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Current point cloud
        self.pointcloud_message = None
        
    def command_callback(self, msg):
        if(msg.data == "detect surfaces"):
            print('Decting surfaces...')
            self.detectSurfaces()
            print('... decting surfaces completed!')

    def camera_callback(self, msg):
        self.pointcloud_message = msg

    def detectSurfaces(self ):
        if self.pointcloud_message == None:
            print('Ups, no pointclound!')
            return
        points = msg_to_open3d(self.pointcloud_message)
        #points = omit_far_points(2, points)
        points = remove_outliers(points, 100, 1)
        planes = detect_planes(points)
        corners = []
        for plane in planes:
            plane_corners = get_corners_of_plane_of_interest(plane)
            for corner in plane_corners:
                corners.append(corner)
        print('Detected {} surfaces!'.format(len(planes)))
        self.publish_corners(corners)
         
    def publish_corners(self, points):
        target_frame = "base_link"  # Define the target frame
        msg = PoseArray()
        msg.header.frame_id = target_frame  # Set the frame_id to the target frame
        for point in points:
            pose_stamped = PointStamped()  # Create a PointStamped for transformation
            pose_stamped.header.frame_id = "camera_depth_optical_frame"  # Original frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.point.x = point[0]
            pose_stamped.point.y = point[1]
            pose_stamped.point.z = point[2]
            # Transform the point
            try:
                transformed_pose = tf2_geometry_msgs.do_transform_point(pose_stamped, self.tf_buffer.lookup_transform(target_frame, pose_stamped.header.frame_id, rclpy.time.Time()))
                msg.poses.append(vector2pose([transformed_pose.point.x, transformed_pose.point.y, transformed_pose.point.z]))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f'Error transforming point: {e}')
        self.publisher_planes.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = PlaneFinder()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()