#include <iostream>
#include <fstream>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "BA/mypcl.hpp"
#include "pose_refine.hpp"
#include <icecream.hpp>

using namespace std;
using namespace Eigen;
double voxel_size, eigen_thr;

void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE*>& feature_map,
               pcl::PointCloud<PointType>::Ptr feature_pts,
               Eigen::Quaterniond q, Eigen::Vector3d t, int f_head, int window_size, double eigen_threshold)
{
	uint pt_size = feature_pts->size();
	for(uint i = 0; i < pt_size; i++)
	{
		PointType& pt = feature_pts->points[i];
		Eigen::Vector3d pt_origin(pt.x, pt.y, pt.z);
		Eigen::Vector3d pt_trans = q * pt_origin + t;

		float loc_xyz[3];
		for(int j = 0; j < 3; j++)
		{
			loc_xyz[j] = pt_trans[j] / voxel_size;
			if(loc_xyz[j] < 0)
				loc_xyz[j] -= 1.0;
		}

		VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
		auto iter = feature_map.find(position);
		if(iter != feature_map.end())
		{
			iter->second->origin_pc[f_head]->push_back(pt_origin);
			iter->second->transform_pc[f_head]->push_back(pt_trans);
		}
		else
		{
			OCTO_TREE* ot = new OCTO_TREE(window_size, eigen_threshold);
			ot->origin_pc[f_head]->push_back(pt_origin);
			ot->transform_pc[f_head]->push_back(pt_trans);

			ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
			ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
			ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
			ot->quater_length = voxel_size / 4.0;
            ot->layer = 0;
			feature_map[position] = ot;
		}
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_refine");
    ros::NodeHandle nh("~");

    ros::Publisher pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/map_surf", 100, true);
    ros::Publisher pub_surf_debug = nh.advertise<sensor_msgs::PointCloud2>("/debug_surf", 100);

    string data_path, bag_name;
    string odom_topic, base_lidar_topic;
    int max_iter, base_lidar;
    double downsmp_base;
    bool load_original = true;
    double pub_hz = 5.0;

    nh.getParam("data_path", data_path);
    nh.getParam("max_iter", max_iter);
    nh.getParam("base_lidar", base_lidar);
    nh.getParam("voxel_size", voxel_size);
    nh.getParam("eigen_threshold", eigen_thr);
    nh.getParam("downsample_base", downsmp_base);
    nh.getParam("load_original", load_original);
    nh.getParam("bag_name", bag_name);
    nh.getParam("odom_topic", odom_topic);
    nh.getParam("base_lidar_topic", base_lidar_topic);
    nh.getParam("pub_hz", pub_hz);

    vector<mypcl::pose> pose_vec;
    size_t pose_size;
    vector<pcl::PointCloud<PointType>::Ptr> base_pc, full_pc;

    if(bag_name.empty()){
        if(load_original)
            pose_vec = mypcl::read_pose(data_path + "original_pose/" + to_string(base_lidar) + ".json");
        else
            pose_vec = mypcl::read_pose(data_path + "pose.json");

        pose_size = pose_vec.size();
        base_pc.resize(pose_size);
        full_pc.resize(pose_size);

        for(size_t i = 0; i < pose_size; i++)
        {
            pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
            pcl::io::loadPCDFile(data_path+to_string(base_lidar)+"/"+to_string(i)+".pcd", *pc);
            base_pc[i] = pc;
            full_pc[i] = pcl::make_shared<pcl::PointCloud<PointType>>();
            *full_pc[i] = *pc;
        }
    }else{
        // load from rosbag
        rosbag::Bag bag(data_path + bag_name, rosbag::bagmode::Read);
        vector<string> topics{odom_topic, base_lidar_topic};
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        // iterate the bag
        for(auto it = view.begin(); it != view.end(); it++){
            auto m = *it;
            if(m.getTopic() == odom_topic){
                nav_msgs::Odometry::ConstPtr pose = m.instantiate<nav_msgs::Odometry>();
                auto q = pose->pose.pose.orientation;
                auto p = pose->pose.pose.position;
                Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
                Eigen::Vector3d et(p.x, p.y, p.z);
                pose_vec.push_back(mypcl::pose(eq, et));
            }else if(m.getTopic() == base_lidar_topic){
                sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
                pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
                pcl::fromROSMsg(*cloud, *pc);
                base_pc.push_back(pc);

                auto tmp = pcl::make_shared<pcl::PointCloud<PointType>>();
                pcl::copyPointCloud(*pc, *tmp);
                full_pc.push_back(tmp);
            }
        }
        pose_size = pose_vec.size(); IC(pose_size);
    }
  
    int loop = 0;
    double avg_time = 0.0;
    ros::Time t_begin, t_end, cur_t;
    sensor_msgs::PointCloud2 debugMsg, colorCloudMsg;
    pcl::PointCloud<PointType>::Ptr pc_debug(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pc_full(new pcl::PointCloud<PointType>);
    
    IC();

    for(; loop < max_iter; loop++)
    {
        cout << "---------------------" << endl;
        cout << "iteration " << loop << endl;
        t_begin = ros::Time::now();
        int window_size = pose_size;
        unordered_map<VOXEL_LOC, OCTO_TREE*> surf_map;
        LM_OPTIMIZER lm_opt(window_size);
        cur_t = ros::Time::now();

        for(size_t i = 0; i < pose_size; i++)
        {
            if(downsmp_base > 0) downsample_voxel(*base_pc[i], downsmp_base);
            cut_voxel(surf_map, base_pc[i], pose_vec[i].q, pose_vec[i].t, i, window_size, eigen_thr);
        }

        for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
            iter->second->recut();

        for(int i = 0; i < window_size; i++)
            assign_qt(lm_opt.poses[i], lm_opt.ts[i], pose_vec[i].q, pose_vec[i].t);
    
        for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
            iter->second->feed_pt(lm_opt);

        lm_opt.optimize();

        for(int i = 0; i < window_size; i++)
            assign_qt(pose_vec[i].q, pose_vec[i].t, lm_opt.poses[i], lm_opt.ts[i]);

        for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
            delete iter->second;
    
        t_end = ros::Time::now();
        cout << "time cost " << (t_end-t_begin).toSec() << endl;
        avg_time += (t_end-t_begin).toSec();

        Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
        pc_full->clear();
        for(size_t i = 0; i < pose_size; i++)
        {
            mypcl::transform_pointcloud(*base_pc[i], *pc_debug, q0.inverse()*(pose_vec[i].t-t0), q0.inverse()*pose_vec[i].q);
            pc_full = mypcl::append_cloud(pc_full, *pc_debug);
        }
        pcl::toROSMsg(*pc_full, debugMsg);
        debugMsg.header.frame_id = "camera_init";
        debugMsg.header.stamp = cur_t;
        pub_surf_debug.publish(debugMsg);
    }
    cout << "---------------------" << endl;
    cout << "complete" << endl;
    cout << "averaged iteration time " << avg_time / (loop+1) << endl;
    mypcl::write_pose(pose_vec, data_path);
    
    Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(),
                            pose_vec[0].q.y(), pose_vec[0].q.z());
    Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
    
    ros::Rate pub_rate(pub_hz);
    for(size_t i = 0; i < pose_size; i++)
    {
        *pc_surf = *(full_pc[i]);
        Eigen::Vector3d et = q0.inverse()*(pose_vec[i].t-t0);
        Eigen::Quaterniond eq = q0.inverse()*pose_vec[i].q;
        cout << i << ": " << eq.coeffs().transpose() << " " << et.transpose() << endl;
        IC(pc_surf->size());
        mypcl::transform_pointcloud(*pc_surf, *pc_surf, et, eq);
        pcl::toROSMsg(*pc_surf, colorCloudMsg);
        colorCloudMsg.header.frame_id = "camera_init";
        colorCloudMsg.header.stamp = cur_t;
        pub_surf.publish(colorCloudMsg);
        pub_rate.sleep();
    }

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}