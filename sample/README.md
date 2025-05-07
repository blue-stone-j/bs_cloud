# cloud downsample and filter

Some algorithms to downsample or filter point cloud.

Cloud files are stored in folder `assets`. You can get more clouds as example from [here](https://github.com/blue-stone-j/assets).

### interface

This is base class for all sample class.

### far_outliers_removal(skip)

### farthest_point_sampling

`farthest_point_sampling` is a greedy algorithm that samples from a point cloud data iteratively. It starts from a random single sample of point. In each iteration, it samples from the rest points that is the farthest from the set of sampled points.

### grid_sample(skip)

### kmeans_sample(skip)

### mirror_reflection_removal(skip)

### normal_space_sample(skip)

Less points are reserved where normal changes slowly, more where faster.

### poisson_disk_sample(skip)

### radius_outlier_removal

Only points that have enough neighbors will be reserved.

### statistical_outlier_removal

`statistical_outlier_removal` removes points that are further away from their neighbors compared to the average for the point cloud.
