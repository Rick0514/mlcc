#include <iostream>
#include <fstream>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>

#include "extrinsic_refine.hpp"
#include "BA/mypcl.hpp"
#include "BA/tools.hpp"
#include <icecream.hpp>

using namespace std;
using namespace Eigen;
double voxel_size, eigen_thr;

void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE*>& feature_map,
               pcl::PointCloud<PointType>::Ptr feature_pts,
               Eigen::Quaterniond q, Eigen::Vector3d t, int f_head,
               int window_size, double eigen_threshold,
               bool is_base_lidar = true)
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
            if(is_base_lidar)
            {
                iter->second->baseOriginPc[f_head]->emplace_back(pt_origin);
                iter->second->baseTransPc[f_head]->emplace_back(pt_trans);
            }
            else
            {
                iter->second->refOriginPc[f_head]->emplace_back(pt_origin);
                iter->second->refTransPc[f_head]->emplace_back(pt_trans);
            }
		}
		else
		{
            OCTO_TREE* ot = new OCTO_TREE(window_size, eigen_threshold);
            if(is_base_lidar)
            {
                ot->baseOriginPc[f_head]->emplace_back(pt_origin);
                ot->baseTransPc[f_head]->emplace_back(pt_trans);
            }
            else
            {
                ot->refOriginPc[f_head]->emplace_back(pt_origin);
                ot->refTransPc[f_head]->emplace_back(pt_trans);
            }

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
    ros::init(argc, argv, "extrinsic_refine");
    ros::NodeHandle nh("~");

    string data_path, bag_name, log_path;
    string odom_topic, base_lidar_topic;
    vector<string> ref_lidar;
    int max_iter, base_lidar;
    double downsmp_base, downsmp_ref;
    double pub_hz = 5.0;

    nh.getParam("data_path", data_path);
    nh.getParam("log_path", log_path);
    nh.getParam("max_iter", max_iter);
    nh.getParam("base_lidar", base_lidar);
    nh.getParam("voxel_size", voxel_size);
    nh.getParam("eigen_threshold", eigen_thr);
    nh.getParam("downsample_base", downsmp_base);
    nh.getParam("downsample_ref", downsmp_ref);

    nh.getParam("bag_name", bag_name);
    nh.getParam("odom_topic", odom_topic);
    nh.getParam("base_lidar_topic", base_lidar_topic);
    nh.getParam("pub_hz", pub_hz);

    vector<mypcl::pose> ref_poses;
    mypcl::pose b2l(Quaterniond(1, 0, 0, 0), Vector3d(0, 0, 0));
    
    // the first string is the ref lidar
    // the third line is the first ref pose
    {
        std::ifstream inf(data_path + "ref.json");
        string line;
        getline(inf, line);
        stringstream ss1(line);
        while (ss1 >> line) ref_lidar.push_back(line);
        
        IC(ref_lidar);
        
        getline(inf, line);
        stringstream ss2(line);
        double tx, ty, tz, w, x, y, z;
        ss2 >> tx >> ty >> tz >> w >> x >> y >> z;
        b2l = mypcl::pose(Eigen::Quaterniond(w, x, y, z),
                Eigen::Vector3d(tx, ty, tz));

        for(int i=0; i<ref_lidar.size(); i++){
            getline(inf, line);
            stringstream ss3(line);
            ss3 >> tx >> ty >> tz >> w >> x >> y >> z;
            ref_poses.push_back(mypcl::pose(Eigen::Quaterniond(w, x, y, z),
                    Eigen::Vector3d(tx, ty, tz)));
        }
    }
    
    for(int ld=0; ld<ref_lidar.size(); ld++){
        cout << "---------lidar "<< ref_lidar[ld] << " -----------" << endl;
        vector<mypcl::pose> pose_vec;
        size_t pose_size;

        vector<pcl::PointCloud<PointType>::Ptr> base_pc, ref_pc;
        if(bag_name.empty()){
            pose_vec = mypcl::read_pose(data_path + "pose.json");
            pose_size = pose_vec.size();
            base_pc.resize(pose_size);
            ref_pc.resize(pose_size);

            for(size_t i = 0; i < pose_size; i++)
            {
                pcl::PointCloud<PointType>::Ptr pc_base(new pcl::PointCloud<PointType>);
                pcl::PointCloud<PointType>::Ptr pc_ref(new pcl::PointCloud<PointType>);
                pcl::io::loadPCDFile(data_path+to_string(base_lidar)+"/"+to_string(i)+".pcd", *pc_base);
                pcl::io::loadPCDFile(data_path+ref_lidar[ld]+"/"+to_string(i)+".pcd", *pc_ref);
                base_pc[i] = pc_base;
                ref_pc[i] = pc_ref;
            }
        }else{
            // load from rosbag
            rosbag::Bag bag(data_path + bag_name, rosbag::bagmode::Read);
            vector<string> topics{odom_topic, base_lidar_topic, ref_lidar[ld]};
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
                }else if(m.getTopic() == ref_lidar[ld]){
                    sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
                    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
                    pcl::fromROSMsg(*cloud, *pc);
                    ref_pc.push_back(pc);
                }
            }
            
            pose_size = pose_vec.size();

            IC(pose_size, base_pc.size(), ref_pc.size());
            // right multiply the p_b2l to each pose in pose_vec
            for(auto& pose : pose_vec)
            {
                pose.q = pose.q * b2l.q;
                pose.t = pose.q * b2l.t + pose.t;
            }
        }

        double avg_time = 0.0;
        ros::Time t_begin, t_end, cur_t;
        // sensor_msgs::PointCloud2 debugMsg, colorCloudMsg;
        // ros::Publisher pub_surf_debug = nh.advertise<sensor_msgs::PointCloud2>("/debug_surf", 10, true);
        // pcl::PointCloud<PointType>::Ptr pc_debug(new pcl::PointCloud<PointType>);
        // pcl::PointCloud<PointType>::Ptr pc_color(new pcl::PointCloud<PointType>);
        int loop = 0;
        for(; loop < max_iter; loop++)
        {
            cout << "---------------------" << endl;
            cout << "iteration " << loop << endl;
            t_begin = ros::Time::now();
            unordered_map<VOXEL_LOC, OCTO_TREE*> surf_map;
            EXTRIN_OPTIMIZER lm_opt(pose_size, 1);
            cur_t = ros::Time::now();

            for(size_t i = 0; i < pose_size; i++)
            {
                pcl::PointCloud<PointType>::Ptr tmp1 = pcl::make_shared<pcl::PointCloud<PointType>>();
                pcl::PointCloud<PointType>::Ptr tmp2 = pcl::make_shared<pcl::PointCloud<PointType>>();
                *tmp1 = *base_pc[i];
                *tmp2 = *ref_pc[i];

                if(downsmp_base > 0) downsample_voxel(*tmp1, downsmp_base);
                if(downsmp_ref > 0) downsample_voxel(*tmp2, downsmp_ref);

                cut_voxel(surf_map, tmp1, pose_vec[i].q, pose_vec[i].t, i, pose_size, eigen_thr);
                
                cut_voxel(surf_map, tmp2, pose_vec[i].q * ref_poses[ld].q,
                            pose_vec[i].q * ref_poses[ld].t + pose_vec[i].t, i, pose_size, eigen_thr, false);
            }

            for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
                iter->second->recut();

            for(size_t i = 0; i < pose_size; i++)
                assign_qt(lm_opt.poses[i], lm_opt.ts[i], pose_vec[i].q, pose_vec[i].t);

            assign_qt(lm_opt.refQs[0], lm_opt.refTs[0], ref_poses[ld].q, ref_poses[ld].t);

            for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
                iter->second->feed_pt(lm_opt);

            lm_opt.optimize();

            assign_qt(ref_poses[ld].q, ref_poses[ld].t, lm_opt.refQs[0], lm_opt.refTs[0]);

            for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
                delete iter->second;

            t_end = ros::Time::now();
            double cost_time = (t_end-t_begin).toSec();
            cout << "time cost " << cost_time << endl;
            avg_time += cost_time;
            
            if(cost_time < 1.0 / pub_hz){
                ros::Duration(1.0 / pub_hz - cost_time).sleep();
            }

            // pc_color->clear();
            // Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
            // Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
            // for(size_t i = 0; i < pose_size; i++)
            // {
            //     mypcl::transform_pointcloud(*base_pc[i], *pc_debug, q0.inverse()*(pose_vec[i].t-t0), q0.inverse()*pose_vec[i].q);
            //     pc_color = mypcl::append_cloud(pc_color, *pc_debug);
            
            //     mypcl::transform_pointcloud(*ref_pc[i], *pc_debug,
            //         q0.inverse()*(pose_vec[i].t-t0)+q0.inverse()*pose_vec[i].q*ref_poses[ld].t,
            //         q0.inverse()*pose_vec[i].q*ref_poses[ld].q);
            //     pc_color = mypcl::append_cloud(pc_color, *pc_debug);
            // }

            // pcl::toROSMsg(*pc_color, debugMsg);
            // debugMsg.header.frame_id = "camera_init";
            // debugMsg.header.stamp = cur_t;
            // pub_surf_debug.publish(debugMsg);

            if(!nh.ok())    return 0;
        }
    
        cout << "---------------------" << endl;
        cout << "complete" << endl;
        cout << "averaged iteration time " << avg_time / (loop+1) << endl;
    }


    {
        // change the ref.json the third line with new ref pose
        std::ifstream inf(data_path + "ref.json");
        string line, each;
        vector<string> first_lines;
        getline(inf, line);
        first_lines.push_back(line);
        getline(inf, line);
        first_lines.push_back(line);
        inf.close();
        
        IC(first_lines);

        std::ofstream out(data_path + "ref.json");
        for(auto e : first_lines)   out << e << endl;

        for(int i=0; i<ref_poses.size(); i++){
            Eigen::Quaterniond q = ref_poses[i].q;
            Eigen::Vector3d t = ref_poses[i].t;
            out << t(0) << " " << t(1) << " " << t(2) << " "
                << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        out.close();
    }

    // mypcl::write_ref(ref_vec, data_path);

    // Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
    // Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
    // pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);

    // ros::Publisher pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/map_surf", 5, true);
    // ros::Publisher pub_surf2 = nh.advertise<sensor_msgs::PointCloud2>("/map_surf2", 5, true);
    // ros::Rate pub_rate(pub_hz);

    // for(size_t i = 0; i < pose_size; i++)
    // {
    //     *pc_surf = *base_pc[i];
    //     mypcl::transform_pointcloud(*pc_surf, *pc_surf, q0.inverse()*(pose_vec[i].t-t0), q0.inverse()*pose_vec[i].q);
    //     pcl::toROSMsg(*pc_surf, colorCloudMsg);
    //     colorCloudMsg.header.frame_id = "camera_init";
    //     colorCloudMsg.header.stamp = cur_t;
    //     pub_surf.publish(colorCloudMsg);
        
    //     *pc_surf = *ref_pc[i];
    //     mypcl::transform_pointcloud(*pc_surf, *pc_surf,
    //         q0.inverse()*(pose_vec[i].t-t0)+q0.inverse()*pose_vec[i].q*ref_vec[0].t,
    //         q0.inverse()*pose_vec[i].q*ref_vec[0].q);
    //     pcl::toROSMsg(*pc_surf, colorCloudMsg);
    //     colorCloudMsg.header.frame_id = "camera_init";
    //     colorCloudMsg.header.stamp = cur_t;
    //     pub_surf2.publish(colorCloudMsg);

    //     pub_rate.sleep();
    // }

    // ros::Rate loop_rate(10);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}