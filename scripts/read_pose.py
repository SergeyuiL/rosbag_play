import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

def read_pose_file_csv(pose_file_path):
    """读取CSV格式的位姿文件"""
    image_ids = []
    pose_matrices = []
    
    with open(pose_file_path, 'r', newline='') as f:
        reader = csv.reader(f, delimiter=' ')
        for row in reader:
            if not row:  # 跳过空行
                continue
                
            try:
                # 将所有元素转换为浮点数（第一个是整数ID）
                row_values = [int(row[0])] + [float(val) for val in row[1:]]
                
                if len(row_values) != 13:  # 1个图像ID + 12个矩阵元素
                    continue
                    
                image_id = row_values[0]
                
                # 创建一个4x4的单位矩阵
                pose_matrix = np.eye(4)
                
                # 填充前三行的矩阵元素
                matrix_elements = row_values[1:]
                for i in range(3):
                    for j in range(4):
                        pose_matrix[i, j] = matrix_elements[i*4 + j]
                
                image_ids.append(image_id)
                pose_matrices.append(pose_matrix)
            except ValueError:
                pass
    
    return image_ids, pose_matrices

def plot_trajectory(pose_matrices, title="Robot Trajectory"):
    """绘制3D轨迹"""
    # 提取位置信息
    positions = np.array([matrix[:3, 3] for matrix in pose_matrices])
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]
    
    # 创建3D图
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制轨迹线
    ax.plot(x, y, z, 'b-', linewidth=2, label='Trajectory')
    
    # 标记起点和终点
    ax.scatter(x[0], y[0], z[0], c='g', marker='o', s=100, label='Start')
    ax.scatter(x[-1], y[-1], z[-1], c='r', marker='o', s=100, label='End')
    
    # 绘制坐标系方向
    # 选择一些关键帧来绘制坐标系，避免图形过于拥挤
    step = max(1, len(pose_matrices) // 20)  # 大约绘制20个坐标系
    for i in range(0, len(pose_matrices), step):
        T = pose_matrices[i]
        pos = T[:3, 3]
        
        # 获取坐标系轴方向
        x_axis = T[:3, 0] * 0.1  # 缩放以便可视化
        y_axis = T[:3, 1] * 0.1
        z_axis = T[:3, 2] * 0.1
        
        # 绘制坐标轴
        ax.quiver(pos[0], pos[1], pos[2], x_axis[0], x_axis[1], x_axis[2], color='r', arrow_length_ratio=0.1)
        ax.quiver(pos[0], pos[1], pos[2], y_axis[0], y_axis[1], y_axis[2], color='g', arrow_length_ratio=0.1)
        ax.quiver(pos[0], pos[1], pos[2], z_axis[0], z_axis[1], z_axis[2], color='b', arrow_length_ratio=0.1)
    
    # 设置图形属性
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    ax.legend()
    
    # 设置坐标轴比例相等
    max_range = np.max([
        np.max(x) - np.min(x),
        np.max(y) - np.min(y),
        np.max(z) - np.min(z)
    ])
    mid_x = (np.max(x) + np.min(x)) * 0.5
    mid_y = (np.max(y) + np.min(y)) * 0.5
    mid_z = (np.max(z) + np.min(z)) * 0.5
    ax.set_xlim(mid_x - max_range * 0.5, mid_x + max_range * 0.5)
    ax.set_ylim(mid_y - max_range * 0.5, mid_y + max_range * 0.5)
    ax.set_zlim(mid_z - max_range * 0.5, mid_z + max_range * 0.5)
    
    plt.tight_layout()
    plt.show()
    
def plot_top_view(pose_matrices, title="Top View (XY Plane)"):
    """绘制俯视图（XY平面）"""
    # 提取位置信息
    positions = np.array([matrix[:3, 3] for matrix in pose_matrices])
    x = positions[:, 0]
    y = positions[:, 1]
    
    # 创建2D图
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # 绘制轨迹线
    ax.plot(x, y, 'b-', linewidth=2, label='Trajectory')
    
    # 标记起点和终点
    ax.scatter(x[0], y[0], c='g', marker='o', s=100, label='Start')
    ax.scatter(x[-1], y[-1], c='r', marker='o', s=100, label='End')
    
    # 绘制机器人朝向（使用箭头）
    step = max(1, len(pose_matrices) // 30)  # 大约绘制30个方向箭头
    for i in range(0, len(pose_matrices), step):
        T = pose_matrices[i]
        pos = T[:3, 3]
        heading = T[:3, 0]  # 假设X轴是前向方向
        
        # 绘制朝向箭头
        ax.arrow(pos[0], pos[1], heading[0]*0.2, heading[1]*0.2, 
                 head_width=0.05, head_length=0.1, fc='r', ec='r')
    
    # 设置图形属性
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    ax.legend()
    ax.axis('equal')  # 确保X和Y轴比例相同
    ax.grid(True)
    
    plt.tight_layout()
    plt.show()

# 主函数
def main():
    pose_file_path = "/home/lsg/catkin_ws/src/rosbag_play/sample/node_pose.txt"  # 替换为你的位姿文件路径
    
    # 读取位姿数据
    image_ids, pose_matrices = read_pose_file_csv(pose_file_path)
    
    if not pose_matrices:
        print("No pose data found!")
        return
    
    print(f"Successfully loaded {len(pose_matrices)} poses")
    
    # 绘制3D轨迹
    plot_trajectory(pose_matrices)
    
    # 绘制俯视图（XY平面）
    plot_top_view(pose_matrices)
    
    # 也可以绘制其他平面视图
    # 侧视图 (XZ平面)
    positions = np.array([matrix[:3, 3] for matrix in pose_matrices])
    plt.figure(figsize=(10, 6))
    plt.plot(positions[:, 0], positions[:, 2], 'b-', linewidth=2)
    plt.scatter(positions[0, 0], positions[0, 2], c='g', marker='o', s=100, label='Start')
    plt.scatter(positions[-1, 0], positions[-1, 2], c='r', marker='o', s=100, label='End')
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('Side View (XZ Plane)')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
