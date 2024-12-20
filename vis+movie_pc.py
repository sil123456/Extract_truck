#!/usr/bin/env python3.11


import numpy as np
import open3d as o3d
import os
import glob
import time
import cv2

def create_line(first, second, num_points=1000, color=[1, 0, 0]):
    x1, y1, z1 = first
    x2, y2, z2 = second

    direction = np.array([x2 - x1, y2 - y1, z2 - z1])
    norm_direction = direction / np.linalg.norm(direction)
    extension_factor = 300  # extend the factor

    point1 = np.array([x1, y1, z1]) - extension_factor * norm_direction
    point2 = np.array([x1, y1, z1]) + extension_factor * norm_direction

    t_values = np.linspace(-extension_factor, extension_factor, num_points)
    line_points = np.array([x1, y1, z1]) + np.outer(t_values, norm_direction)

    line_colors = np.tile(color, (line_points.shape[0], 1))


    points = np.vstack((point1, point2))
    lines = [[0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([color])

    # print(line_points, line_colors)

    return line_points, line_colors, line_set



# 22539,17.3364276886,-41.2215881348,-3.12704324722
# 3633,18.8406066895,38.6632156372,-0.750732719898
# 2728,15.3245296478,43.4921302795,-0.804904997349
# 22540,13.7527103424,-37.7852630615,-3.28256821632
# 2381,11.385052681,27.7458324432,-1.57175338268
# 5296,11.0873851776,11.7368850708,-2.05514168739
# 781,7.921479702,39.3215789795,-1.40072619915
# 23302,6.72330999374,-26.0533790588,-3.42489933968


line1 = [17.3364276886, -41.2215881348, -3.12704324722]
line2 = [18.8406066895, 38.6632156372, -0.750732719898]
line3 = [15.3245296478, 43.4921302795, -0.804904997349]
line4 = [13.7527103424, -37.7852630615, -3.28256821632]
line5 = [11.385052681, 27.7458324432, -1.57175338268]
line6 = [11.0873851776, 11.7368850708, -2.05514168739]
line7 = [7.921479702, 39.3215789795, -1.40072619915]
line8 = [6.72330999374, -26.0533790588, -3.42489933968]


folder_path = r"Bobtail\\bobtail_1\\" 

pcd_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.pcd')])




vis = o3d.visualization.Visualizer()
vis.create_window()


ctr = vis.get_view_control()




# 创建视频编写器

out_path = r"movie\\bobtail_13_137.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 可根据需要选择其他编码器
fps = 1  # fps
width, height = 4000, 2000  # 视频帧大小
video_writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))



point_cloud_o3d = o3d.geometry.PointCloud()
line_sets = []

start_time = time.time()
prev_sec = -1
first_call = True


render_option = vis.get_render_option()
render_option.point_size = 1 

for _, file_name in enumerate(pcd_files):

    file_path = os.path.join(folder_path, file_name)
    pcd = o3d.io.read_point_cloud(file_path)
    
    if first_call:
        line_points_1, line_colors_1, line_set_1 = create_line(line1, line2)
        line_points_2, line_colors_2, line_set_2 = create_line(line3, line4)
        line_points_3, line_colors_3, line_set_3 = create_line(line5, line6)
        line_points_4, line_colors_4, line_set_4 = create_line(line7, line8)

        line_sets = [line_set_1, line_set_2, line_set_3, line_set_4]

        for line_set in line_sets:
            vis.add_geometry(line_set)


    point_cloud_o3d.points = o3d.utility.Vector3dVector(pcd.points)
    point_cloud_o3d.colors = pcd.colors


    ctr.set_up((1, 0, 0))  # set the negative direction of the y-axis as the up direction
    ctr.set_front((-0.5, 0, 0.5))  # set the positive direction of the x-axis toward you
    # ctr.set_front((0, 0, 0.5))  # set the positive direction of the x-axis toward you
    ctr.set_lookat((0, 0, 0))  # set the original point as the center point of the window
    ctr.set_zoom(0.07)



    vis.update_geometry(point_cloud_o3d)

    if first_call: 
        vis.add_geometry(point_cloud_o3d)
        first_call = False
    else:
        vis.update_geometry(point_cloud_o3d)
    if not vis.poll_events():
        break


    img = vis.capture_screen_float_buffer()
    img = (np.array(img) * 255).astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 通道转换
    img = cv2.resize(img, (width, height))
    video_writer.write(img)


    vis.update_renderer()
    time.sleep(1/fps)  


vis.close()
vis.destroy_window()
