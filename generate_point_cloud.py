import os
import open3d as o3d
import argparse

def generate_point_cloud(color_images_path, depth_images_path, path, visualization):
	color_images = sorted(os.listdir(color_images_path))
	depth_images = sorted(os.listdir(depth_images_path))

	cam1 = o3d.camera.PinholeCameraIntrinsic()
	cam1.intrinsic_matrix =  [[1070.58, 0.00, 927.269] , [0.00, 1069.126, 545.76], [0.00, 0.00, 1.00]]
	#cam1.intrinsic_matrix =  [[112, 0.00, 112] , [0.00, 112, 112], [0.00, 0.00, 1.00]]

	if os.path.isdir(path) == False:
		os.mkdir(path)

	for i in range(len(color_images)):
		color_image_path = os.path.join(color_images_path, color_images[i])
		depth_image_path = os.path.join(depth_images_path, depth_images[i])
		
		color_image = o3d.io.read_image(color_image_path)
		depth_image = o3d.io.read_image(depth_image_path)
		
		rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
			color_image,
			depth_image
		)    
		pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
			rgbd_image,
			cam1)
		
		# Flip it, otherwise the pointcloud will be upside down
		pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
		
		if visualization == True:
			o3d.visualization.draw_geometries([pcd])
		o3d.io.write_point_cloud(f"{path}/{i}.pcd", pcd)

def generate_point_cloud_from_depth(color_images_path, depth_images_path, path, visualization):
	color_images = sorted(os.listdir(color_images_path))
	depth_images = sorted(os.listdir(depth_images_path))
	
	cam1 = o3d.camera.PinholeCameraIntrinsic()
	cam1.intrinsic_matrix =  [[1070.58, 0.00, 927.269] , [0.00, 1069.126, 545.76], [0.00, 0.00, 1.00]]
	
	cx_rgb = cam1.intrinsic_matrix[0,2]
	cy_rgb = cam1.intrinsic_matrix[1,2]
	fx_rgb = cam1.intrinsic_matrix[0,0]
	fy_rgb = cam1.intrinsic_matrix[1,1]
	
	for i in range(len(depth_images)):
		color_image_path = os.path.join(color_images_path, color_images[i])
		depth_image_path = os.path.join(depth_images_path, depth_images[i])
		
		color_image = o3d.io.read_image(color_image_path)
		depth_image = o3d.io.read_image(depth_image_path)
		
		depth = np.array(depth_image, dtype=np.int)/1000
		#plt.imshow(depth); plt.show()
		[nrows, ncols] = depth.shape
		
		pcloud = np.zeros([nrows, ncols, 3])
		xgrid, ygrid = np.meshgrid(np.arange(0,ncols,1),np.arange(0,nrows,1)) 
		xgrid = xgrid - cx_rgb
		ygrid = ygrid - cy_rgb
		
		pcloud[:,:,0] = (xgrid*depth)/fx_rgb;
		pcloud[:,:,1] = (ygrid*depth)/fy_rgb;
		pcloud[:,:,2] = depth
		
		pcloud = pcloud.reshape(-1,3)
			
		if visualization == True:
			pcd = o3d.geometry.PointCloud()
			pcd.points = o3d.utility.Vector3dVector(pcloud)
			pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
			o3d.visualization.draw_geometries([pcd])
		
		o3d.io.write_point_cloud(f"{path}/{i}.pcd", pcd)
		'''
		fig = plt.figure()
		ax = plt.axes(projection='3d')
		ax.scatter3D(pcloud[:,:,0], pcloud[:,:,1], pcloud[:,:,2])
		plt.show()
		'''
def main():
	parser = argparse.ArgumentParser()
	
	parser.add_argument("-cp", "--color_images_path", default= './Home_002_1/jpg_rgb/', help="Specify the folder containing color images")
	parser.add_argument("-dp", "--depth_images_path", default= './Home_002_1/high_res_depth/', help="Specify the folder containing depth images")
	parser.add_argument("-o", "--output_path", default = './point_clouds', help="specify the folder where you want to save the point clouds")
	parser.add_argument("-f", "--create_from_depth_only", type=bool, default = False, help="Specify whether you want to use open3D or create from \
		depth only using numpy")
	parser.add_argument("-v", "--visualization", type=bool, default=False, help="Visualize on/off")
	
	args = parser.parse_args()
	
	if args.create_from_depth_only == False:
		generate_point_cloud(args.color_images_path, args.depth_images_path, args.output_path, args.visualization)
	else:
		generate_point_cloud_from_depth(args.color_images_path, args.depth_images_path, args.output_path, args.visualization)
		
if __name__ == "__main__":
	main()
	
	
