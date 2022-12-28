import argparse
import numpy as np
import open3d


def generate_colors(N):
    base_colors = np.array(
        [[255,   0,   0],
         [0, 255,   0],
         [0,   0, 255],
         [255, 255,   0],
         [255,   0, 255],
         [0, 255, 255]])
    if (N < len(base_colors)):
        return base_colors[:N, :]
    else:
        additinal_colors = np.clip(np.random.random([N - len(base_colors), 3]), 0, 255)
        return np.vstack([base_colors, additinal_colors])


def label(filename):
    with open(filename, "rb") as f:
        data = np.fromfile(f, dtype=np.float32)
    assert data.shape[0] % 4 == 0
    matrix_data = data.reshape(data.shape[0] // 4, 4)

    labels = matrix_data[:, 3].astype(int)
    num_labels = np.max(labels) + 1
    colors = generate_colors(num_labels)

    point_cloud = open3d.geometry.PointCloud()
    for i in range(matrix_data.shape[0]):
        point_cloud.points.append(matrix_data[i, :3])
        point_cloud.colors.append(colors[labels[i]])

    open3d.visualization.draw_geometries(
        [point_cloud],
        zoom=0.3,
        front=[0, 1, 0],
        lookat=[0, 0, 0],
        up=[0, 0, 1])


def compare(filenames):
    colors = generate_colors(len(filenames))
    point_clouds = []

    for filename, color in zip(filenames, colors):
        with open(filename, "rb") as f:
            data = np.fromfile(f, dtype=np.float32)
        assert data.shape[0] % 4 == 0
        matrix_data = data.reshape(data.shape[0] // 4, 4)

        point_cloud = open3d.geometry.PointCloud()
        for i in range(matrix_data.shape[0]):
            point_cloud.points.append(matrix_data[i, :3])
            point_cloud.colors.append(color)

        point_clouds.append(point_cloud)

    open3d.visualization.draw_geometries(
        point_clouds,
        zoom=0.3,
        front=[0, 1, 0],
        lookat=[0, 0, 0],
        up=[0, 0, 1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", type=str, choices=["label", "comp"], help="\"label\" or \"comp\"")
    parser.add_argument("filenames", type=str, help="Path to point cloud binary file.", nargs="+")
    args = parser.parse_args()
    mode = args.mode

    if (mode == "label"):
        label(args.filenames[0])
    elif (mode == "comp"):
        if (len(args.filenames) < 2):
            print("2 files are required at least.")
            exit(1)
        compare(args.filenames)


if __name__ == "__main__":
    main()
