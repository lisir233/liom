#include "scanmatch.h"
namespace liom{
ScanMatch::ScanMatch(/* args */)
{
    initialized_ = false;
    InitParams();
}
ScanMatch::~ScanMatch()
{

}
/**
 * 将雷达的数据格式转成 csm 需要的格式
 */
void ScanMatch::LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp)
{
    unsigned int n = scan_msg->ranges.size();
    // 调用csm里的函数进行申请空间
    ldp = ld_alloc_new(n);

    for (unsigned int i = 0; i < n; i++)
    {
        // calculate position in laser frame
        double r = scan_msg->ranges[i];

        if (r > scan_msg->range_min && r < scan_msg->range_max)
        {
            // 填充雷达数据
            ldp->valid[i] = 1;
            ldp->readings[i] = r;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1; // for invalid range
        }

        ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
        ldp->cluster[i] = -1;
    }

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->estimate[0] = 0.0;
    ldp->estimate[1] = 0.0;
    ldp->estimate[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

/*
 * csm的参数初始化
 */
void ScanMatch::InitParams()
{
    // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)

    // Maximum angular displacement between scans
    input_.max_angular_correction_deg = 45.0;

    // Maximum translation between scans (m)
    input_.max_linear_correction = 0.2;

    // Maximum ICP cycle iterations
    input_.max_iterations = 10;

    // A threshold for stopping (m)
    input_.epsilon_xy = 0.000001;

    // A threshold for stopping (rad)
    input_.epsilon_theta = 0.000001;

    // Maximum distance for a correspondence to be valid
    input_.max_correspondence_dist = 1.0;

    // Noise in the scan (m)
    input_.sigma = 0.010;

    // Use smart tricks for finding correspondences.
    input_.use_corr_tricks = 1;

    // Restart: Restart if error is over threshold
    input_.restart = 0;

    // Restart: Threshold for restarting
    input_.restart_threshold_mean_error = 0.01;

    // Restart: displacement for restarting. (m)
    input_.restart_dt = 1.0;

    // Restart: displacement for restarting. (rad)
    input_.restart_dtheta = 0.1;

    // Max distance for staying in the same clustering
    input_.clustering_threshold = 0.25;

    // Number of neighbour rays used to estimate the orientation
    input_.orientation_neighbourhood = 20;

    // If 0, it's vanilla ICP
    input_.use_point_to_line_distance = 1;

    // Discard correspondences based on the angles
    input_.do_alpha_test = 0;

    // Discard correspondences based on the angles - threshold angle, in degrees
    input_.do_alpha_test_thresholdDeg = 20.0;

    // Percentage of correspondences to consider: if 0.9,
    // always discard the top 10% of correspondences with more error
    input_.outliers_maxPerc = 0.90;

    // Parameters describing a simple adaptive algorithm for discarding.
    //  1) Order the errors.
    //  2) Choose the percentile according to outliers_adaptive_order.
    //     (if it is 0.7, get the 70% percentile)
    //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
    //     with the value of the error at the chosen percentile.
    //  4) Discard correspondences over the threshold.
    //  This is useful to be conservative; yet remove the biggest errors.
    input_.outliers_adaptive_order = 0.7;

    input_.outliers_adaptive_mult = 2.0;

    // If you already have a guess of the solution, you can compute the polar angle
    // of the points of one scan in the new position. If the polar angle is not a monotone
    // function of the readings index, it means that the surface is not visible in the
    // next position. If it is not visible, then we don't use it for matching.
    input_.do_visibility_test = 0;

    // no two points in laser_sens can have the same corr.
    input_.outliers_remove_doubles = 1;

    // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    input_.do_compute_covariance = 0;

    // Checks that find_correspondences_tricks gives the right answer
    input_.debug_verify_tricks = 0;

    // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
    // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
    input_.use_ml_weights = 0;

    // If 1, the field 'readings_sigma' in the second scan is used to weight the
    // correspondence by 1/sigma^2
    input_.use_sigma_weights = 0;
}
/**
 * 使用PLICP进行帧间位姿的计算
 */
void ScanMatch::ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time)
{
    // CSM is used in the following way:
    // The scans are always in the laser frame
    // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
    // The new scan (currLDPScan) has a pose equal to the movement
    // of the laser in the laser frame since the last scan
    // The computed correction is then propagated using the tf machinery

    input_.laser_ref = prev_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;



    // 调用csm里的函数进行plicp计算帧间的匹配，输出结果保存在output里
    sm_icp(&input_, &output_);

    if (output_.valid)
    {
        // std::cout << "transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", " 
        //     << output_.x[2] * 180 / M_PI << ")" <<"iterations"<<output_.iterations<< std::endl;
    }
    else
    {
        std::cout << "not Converged" << std::endl;
    }

    // 删除prev_ldp_scan_，用curr_ldp_scan进行替代
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    last_icp_time_ = time;
    
}


void ScanMatch::Calculate(const tf2::Transform &first_guess_pose,const sensor_msgs::LaserScan::ConstPtr &scan_msg,tf2::Transform&  result_pose)
{
    tf2::Vector3 guess_origin = first_guess_pose.getOrigin();
    tf2::Matrix3x3  guess_rotation = first_guess_pose.getBasis();
    double x = guess_origin.getX();
    double y = guess_origin.getY();
    double roll,pitch,yaw ;
    guess_rotation.getRPY(roll, pitch, yaw);

    input_.first_guess[0] = x;
    input_.first_guess[1] = y;
    input_.first_guess[2] = yaw;
     // 如果是第一帧数据，首先进行初始化
    if (!initialized_)
    {
        // 将雷达各个角度的sin与cos值保存下来，以节约计算量
        CreateCache(scan_msg);

        // 将 prev_ldp_scan_,last_icp_time_ 初始化
        LaserScanToLDP(scan_msg, prev_ldp_scan_);
        last_icp_time_ = scan_msg->header.stamp;
        initialized_ = true;
        return;
    }
    
    LDP curr_ldp_scan;
    LaserScanToLDP(scan_msg, curr_ldp_scan);
    ScanMatchWithPLICP(curr_ldp_scan, scan_msg->header.stamp);
    tf2::Vector3 result_origin;
    tf2::Matrix3x3  result_rotation;
    result_origin.setX(output_.x[0]);
    result_origin.setY(output_.x[1]);
    result_rotation.setEulerYPR(0,0,output_.x[2]);

    result_pose.setOrigin(result_origin);
    result_pose.setBasis(result_rotation);

}
}
