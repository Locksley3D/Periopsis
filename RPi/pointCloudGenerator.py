import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting

CSV_FILE = 'scan_data.csv'  # Make sure this matches your CSV file name
PLY_FILE = 'point_cloud.ply'  # Output file name

def polar_to_cartesian(rotation_deg, angle_deg, distance_mm):
    """Convert spherical coordinates to Cartesian (in mm)"""
    rot_rad = np.radians(rotation_deg)  # elevation
    ang_rad = np.radians(angle_deg)     # azimuth (lidar scan angle)
    
    x = distance_mm * np.sin(rot_rad) * np.cos(ang_rad)
    y = distance_mm * np.sin(rot_rad) * np.sin(ang_rad)
    z = distance_mm * np.cos(rot_rad)
    return x, y, z

def write_ply(filename, xs, ys, zs):
    """Write 3D point cloud data to a PLY file"""
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(xs)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y, z in zip(xs, ys, zs):
            f.write(f"{x:.3f} {y:.3f} {z:.3f}\n")
    print(f"Exported to {filename}")

def main():
    # Load the CSV
    df = pd.read_csv(CSV_FILE)

    # Normalize column names
    df.columns = [col.strip().lower() for col in df.columns]

    # Match column names
    rotation_col = [col for col in df.columns if "rotation" in col][0]
    angle_col = [col for col in df.columns if "angle" in col][0]
    distance_col = [col for col in df.columns if "distance" in col][0]

    # Ensure numeric types
    df = df.astype({
        rotation_col: float,
        angle_col: float,
        distance_col: float
    })

    xs, ys, zs = [], [], []
    for _, row in df.iterrows():
        # FLIPPING rotation and angle
        x, y, z = polar_to_cartesian(row[angle_col], row[rotation_col], row[distance_col])
        xs.append(x)
        ys.append(y)
        zs.append(z)

    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xs, ys, zs, c=zs, cmap='viridis', s=1)

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('3D Point Cloud from LIDAR + Rotation Data')
    plt.tight_layout()
    plt.show()

    # Export to PLY
    write_ply(PLY_FILE, xs, ys, zs)

if __name__ == '__main__':
    main()
