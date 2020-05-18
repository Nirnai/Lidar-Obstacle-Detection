import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import hdbscan
import time

pcd = o3d.io.read_point_cloud("test_pcd.pcd")

# labels = np.array(pcd.cluster_dbscan(1, 10))
# max_label = labels.max()


# o3d.visualization.draw_geometries([pcd])



start = time.clock()
clusterer = hdbscan.HDBSCAN(min_cluster_size=10).fit(np.asarray(pcd.points))
elapsed = time.clock()
elapsed = elapsed-start
print(elapsed)

labels = clusterer.labels_
max_label = clusterer.labels_.max()


print(max_label)

colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([pcd])
