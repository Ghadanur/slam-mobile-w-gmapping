#include <iostream>
#include <cmath>
#include <vector>
#include <sstream>

#include <Eigen/Core>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>



typedef pcl::PointXYZ PointT;

struct PlaneCluster
{
    pcl::PointCloud<PointT>::Ptr cloud;
    Eigen::Vector4f centroid;

    Eigen::Vector3f normal;
    float d;
};

bool canMergePlanes(const PlaneCluster& a,
                    const PlaneCluster& b,
                    float Tnorm,
                    float Tdist)
{
    // Angle similarity (normal difference)
    float dn = 1.0f - std::abs(a.normal.dot(b.normal));

    // Plane distance similarity
    float dd = std::abs(a.d - b.d);

    return (dn < Tnorm && dd < Tdist);
}
    float minDistanceBetweenClouds(
        pcl::PointCloud<PointT>::Ptr a,
        pcl::PointCloud<PointT>::Ptr b)
    {
        pcl::search::KdTree<PointT> tree;
        tree.setInputCloud(a);

        float min_dist = std::numeric_limits<float>::max();

        for (const auto& pa : a->points)
             for (const auto& pb : b->points)
                    min_dist = std::min(min_dist,
                        std::sqrt(
                            (pa.x - pb.x)*(pa.x - pb.x) +
                            (pa.y - pb.y)*(pa.y - pb.y) +
                            (pa.z - pb.z)*(pa.z - pb.z)
                        ));

            return min_dist;
    }


int main()
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr embossed_cloud(new pcl::PointCloud<PointT>);
    
    pcl::search::KdTree<PointT>::Ptr tree2(new pcl::search::KdTree<PointT>);
    std::vector<pcl::PointCloud<PointT>::Ptr> fragments;


    pcl::PCDReader reader;
    pcl::PCDWriter writer;

    std::string path = "/home/ghada/pcd/";
    std::string input_file = path + "save_cloud_0.pcd";

    if (reader.read(input_file, *cloud) == -1 || cloud->empty())
    {
        std::cerr << "Failed to load input cloud\n";
        return -1;
    }

    // --- Voxel Grid ---
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.03f, 0.03f, 0.03f);
    vg.filter(*voxel_cloud);

    // --- Plane segmentation ---
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setMaxIterations(1000);
    seg.setInputCloud(voxel_cloud);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty())
    {
        std::cerr << "No plane found\n";
        return -1;
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(voxel_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);
    
    
    // --- Extract embossed LED points ---
    float a = coeff->values[0];
    float b = coeff->values[1];
    float c = coeff->values[2];
    float d = coeff->values[3];

    float norm = std::sqrt(a*a + b*b + c*c);
    float emboss_thresh = 0.034f;   // 🔴 tune this
    
    //float max_d = 0, min_d = 1e9; //temp
    for (const auto& p : plane_cloud->points)
    {
        float dist = std::fabs(a*p.x + b*p.y + c*p.z + d) / norm;
        if (dist > emboss_thresh)
        {
            embossed_cloud->points.push_back(p);      
        }
       // max_d = std::max(max_d, dist); //temp
       // min_d = std::min(min_d, dist);       //temp   
       // std::cout << "Plane dist min=" << min_d
      //            << " max=" << max_d << std::endl; //temp
    }
    
    embossed_cloud->width = embossed_cloud->points.size();
    embossed_cloud->height = 1;
    embossed_cloud->is_dense = true;
    
    std::cout << "[DEBUG] Embossed points = "
          << embossed_cloud->size() << std::endl;

    if (embossed_cloud->empty())
    {
        std::cerr << "No embossed points → cannot cluster\n";
        return 0;
    }

    tree2->setInputCloud(embossed_cloud);

    std::vector<pcl::PointIndices> led_clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.09);   // DEBUG .099
    ec.setMinClusterSize(20);        // DEBUG 11
    ec.setMaxClusterSize(100);
    ec.setSearchMethod(tree2);
    ec.setInputCloud(embossed_cloud);
    ec.extract(led_clusters);

    std::cout << "[DEBUG] LED clusters found = "
              << led_clusters.size() << std::endl;
              
    std::vector<PlaneCluster> clusters;

    for (const auto& c : led_clusters)
    {
        pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>);
        for (int idx : c.indices)
            temp->points.push_back(embossed_cloud->points[idx]);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*temp, centroid);
        
        PlaneCluster pc;
        pc.cloud = temp;
        pc.centroid = centroid;

// --- Estimate plane for this LED cluster ---
        pcl::SACSegmentation<PointT> seg_led;
        pcl::ModelCoefficients::Ptr coeff_led(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_led(new pcl::PointIndices);

        seg_led.setOptimizeCoefficients(true);
        seg_led.setModelType(pcl::SACMODEL_PLANE);
        seg_led.setMethodType(pcl::SAC_RANSAC);
        seg_led.setDistanceThreshold(0.01);
        seg_led.setInputCloud(temp);
        seg_led.segment(*inliers_led, *coeff_led);

// plane parameters
        pc.normal = Eigen::Vector3f(
            coeff_led->values[0],
            coeff_led->values[1],
            coeff_led->values[2]
        );
        float n = pc.normal.norm();
        pc.d = coeff_led->values[3] / n;

        clusters.push_back(pc);
        fragments.push_back(temp); 

    }
    

    int N = fragments.size();
    std::vector<std::vector<int>> graph(N);
    
    float merge_dist = 0.40f;  // LED edge spacing (tune 0.05–0.07)
     
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            float d = minDistanceBetweenClouds(fragments[i], fragments[j]);
            std::cout << "dist(" << i << "," << j << ") = " << d << std::endl;
            
            if (d < merge_dist)
            {
                graph[i].push_back(j);
                graph[j].push_back(i);
            }
        }
    }
    std::vector<bool> visited(N, false);
    std::vector<pcl::PointCloud<PointT>::Ptr> final_leds;

    for (int i = 0; i < N; ++i)
    {
        if (visited[i]) continue;

        pcl::PointCloud<PointT>::Ptr led(new pcl::PointCloud<PointT>);
        std::vector<int> stack = {i};
        visited[i] = true;

        while (!stack.empty())
        {
            int u = stack.back();
            stack.pop_back();

            *led += *fragments[u];

            for (int v : graph[u])
            {
                if (!visited[v])
                {
                    visited[v] = true;
                    stack.push_back(v);
                }
            }
        }

        final_leds.push_back(led);
    }
    
    int id = 0;
    for (auto& led : final_leds)
    {
        led->width = led->size();
        led->height = 1;

        std::stringstream ss;
        ss << "/home/ghada/pcd/LED_" << id++ << ".pcd";
        pcl::io::savePCDFileBinary(ss.str(), *led);
    }

    std::cout << "==============================\n";
    std::cout << "✅ TOTAL LEDs = "
              << final_leds.size() << std::endl;
    std::cout << "==============================\n";
    
    for (size_t i = 0; i < clusters.size(); ++i)
    {
        std::cout << "Normal " << i << ": "
                  << clusters[i].normal.transpose() << std::endl;
    }
    
    
    std::cout << "Embossed LED points: " << embossed_cloud->size() << std::endl;
    // --- Save results ---
    writer.write(path + "plane.pcd", *plane_cloud, true);
    writer.write(path + "embossed_led.pcd", *embossed_cloud, true);

    return 0;
}

