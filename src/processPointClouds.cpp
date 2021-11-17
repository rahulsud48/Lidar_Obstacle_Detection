// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>), cloud_f(new pcl::PointCloud<PointT>);

    for(int index: inliers->indices){
        cloud_p->points.push_back(cloud->points[index]);
    }

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative (true);
    extract.filter(*cloud_f);

    // extract.setNegative (true);
    // extract.filter (*cloud_f);
    // cloud.swap (cloud_f);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	


    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcdBlob(std::string file)
{

    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
    typename pcl::PointCloud<PointT>::Ptr cloud (new typename pcl::PointCloud<PointT>);
    pcl::PCDReader reader;
    reader.read (file, *cloud_blob);

    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


// RANSAC for Segmentation prepared by Rahul
template<typename PointT>
void ProcessPointClouds<PointT>::crossProduct(std::vector<float>& v_A, std::vector<float>& v_B, std::vector<float>& v_C){
    v_C[0] = v_A[1]*v_B[2] - v_A[2]*v_B[1];
    v_C[1] = -(v_A[0]*v_B[2] - v_A[2]*v_B[0]);
    v_C[2] = v_A[0]*v_B[1] - v_A[1]*v_B[0];
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations--){

		// Randomly pick 2 points

		std::unordered_set<int> inliers;
		while (inliers.size() < 4){
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        std::vector<float> v1{x2-x1, y2-y1, z2-z1}, v2{x3-x1, y3-y1, z3-z1};
        std::vector<float> v3(3);

        // v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
        // v3[1] = -(v1[0]*v2[2] - v1[2]*v2[0]);
        // v3[2] = v1[0]*v2[1] - v1[1]*v2[0];

        crossProduct(v1,v2,v3);

		float a = v3[0];
		float b = v3[1];
		float c = v3[2];
        float d = -(a*x1 + b*y1 + c*z1);

		for(int index = 0; index < cloud-> points.size(); index++){
			if(inliers.count(index)>0){
				continue;
			}

			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
            float z4 = point.z;

			float D = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);

			if(D<=distanceTol){
				inliers.insert(index);
			}


		}

		if(inliers.size()>inliersResult.size()){
			inliersResult = inliers;
		}

	}
	
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_local(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
	std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::ClusterRecursive(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int curr_index, std::vector<bool>& processed, std::vector<int>& cluster)
{
    processed[curr_index] = true;
    cluster.push_back(curr_index);

    std::vector<int> nearby = tree->search(points[curr_index], distanceTol);

    for (int i: nearby)
    {
        if (!processed[i])
        ClusterRecursive(points, tree, distanceTol, i, processed, cluster);
    }

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while (i < points.size())
    {
        if (processed[i]){
            i++;
            continue;
        }
        std::vector<int> cluster;
        ClusterRecursive(points, tree, distanceTol, i, processed, cluster);

        clusters.push_back(cluster);

        i++;
    }
        return clusters;

}

// template<typename PointT>
// pcl::PointXYZXI ProcessPointClouds<PointT>::CreateData(std::vector<float> v)
// {

  	

//     typename PointT point;
//     point.x = v[0];
//     point.y = v[1];
//     point.z = v[2];



//   	return point;

// }

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_local(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // My solution
    std::vector<std::vector<float>> points_cloud;
    for (int index = 0; index < cloud->points.size(); index++){
        PointT point = cloud->points[index];
        std::vector<float> v{point.x, point.y, point.z};
        points_cloud.push_back(v);
    }

    // std::cout<<"***********************";
    // for (int index = 0; index < points_cloud.size(); index++){
    //     for (int i = 0; i < 3; i++){
    //         std::cout<<points_cloud[index][i];
    //     }
    // }
    
    // std::cout<<"***********************";

	KdTree* tree_ = new KdTree;
  
    for (int i=0; i<points_cloud.size(); i++){
    	tree_->insert(points_cloud[i],i); 
    }
    std::vector<std::vector<int>> clusters = euclideanCluster(points_cloud, tree_, clusterTolerance);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters_all;
  	for(std::vector<int> cluster : clusters)
  	{
  		typename pcl::PointCloud<PointT>::Ptr cloudCluster(new typename pcl::PointCloud<PointT>());
        if (cluster.size() >= minSize && cluster.size() <= maxSize){
            for(int indice: cluster){
                cloudCluster->points.push_back(cloud->points[indice]);
                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                cloudCluster->is_dense = true;
                clusters_all.push_back(cloudCluster);
            }
        }


  	}



    

    // // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(cloud);

    // std::vector<pcl::PointIndices> clusterIndices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(clusterTolerance);
    // ec.setMinClusterSize(minSize);
    // ec.setMaxClusterSize(maxSize);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(clusterIndices);

    // for(pcl::PointIndices getIndices: clusterIndices){
    //     typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

    //     for(int index : getIndices.indices){
    //         cloudCluster->points.push_back(cloud->points[index]);
    //     }

    //     cloudCluster->width = cloudCluster->points.size();
    //     cloudCluster->height = 1;
    //     cloudCluster->is_dense = true;
    //     clusters.push_back(cloudCluster);
    // }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters_all.size() << " clusters" << std::endl;

    return clusters_all;
}