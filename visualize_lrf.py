import open3d as o3d
import numpy as np

if __name__ == "__main__":
	mesh = o3d.io.read_triangle_mesh('../ModelNet10/chair/test/chair_0890.off')  # Change this line according to where your mesh file is
	geometries = [mesh]

	file1 = open('lrf_file.txt', 'r') 
	lines = file1.readlines()

	selected_points = np.linspace(50,1650,50).astype(int)  # randomly selected points

	for i in selected_points:
		lrf = lines[i].split(" ")
		orig = [float(lrf[0]), float(lrf[1]), float(lrf[2])]
		vecs = np.zeros((3, 3))
		vecs[0] = [float(lrf[3]), float(lrf[4]), float(lrf[5])]  # x
		vecs[1] = [float(lrf[6]), float(lrf[7]), float(lrf[8])]  # y
		vecs[2] = [float(lrf[9]), float(lrf[10]), float(lrf[11][:-1])] # z - don't take last char since it is \n
		
		frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 3)
		rotation = vecs.transpose()
		frame.rotate(rotation, [0,0,0])
		frame.translate(orig)
		geometries.append(frame)

	o3d.visualization.draw_geometries(
		geometry_list= geometries,
		window_name= "Local reference frames", width= 800, height= 600
	)