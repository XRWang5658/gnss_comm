#include <ros/ros.h>
#include <vector>
#include <map>

#include "gnss_ros.hpp"
#include "gnss_spp.hpp"
#include "gnss_utility.hpp"
#include "gnss_constant.hpp"

using namespace gnss_comm;

/**
 * @brief 临时的转换函数，用于将Python脚本生成的自定义ID转换为gnss_comm兼容的ID。
 * * @param custom_sat_id 从Python脚本接收到的、经过自定义偏移量处理的卫星ID。
 * @return uint32_t 一个与gnss_comm内部satsys函数兼容的、连续的卫星ID。
 * 如果输入ID无法识别，则返回0。
 */
uint32_t convert_custom_id_to_gnss_comm_id(uint32_t custom_sat_id)
{
    uint32_t sys = SYS_NONE;
    uint32_t svid = 0;

    // --- 第1步: 解码 - 根据Python脚本的逻辑，反向解析出星座和原始SVID ---
    if (custom_sat_id >= 1 && custom_sat_id <= 32) { // GPS
        sys = SYS_GPS;
        svid = custom_sat_id;
    } else if (custom_sat_id >= 101 && custom_sat_id <= 124) { // GLONASS (custom_id = svid + 100)
        sys = SYS_GLO;
        svid = custom_sat_id - 100;
    } else if (custom_sat_id >= 201 && custom_sat_id <= 263) { // BeiDou (custom_id = svid + 200)
        sys = SYS_BDS;
        svid = custom_sat_id - 200;
    } else if (custom_sat_id >= 301 && custom_sat_id <= 336) { // Galileo (custom_id = svid + 300)
        sys = SYS_GAL;
        svid = custom_sat_id - 300;
    } 
    // 注意: 您的Python脚本对QZSS的处理逻辑与其他系统不同，这里也需要对应
    else if (custom_sat_id >= 193 && custom_sat_id <= 202) { // QZSS
        // QZSS是一个特殊情况，您的Python脚本没有加偏移量
        // 但为了让gnss_comm正确处理，我们仍需知道它是哪个系统
        // 这里我们暂时将其归类为一个自定义的系统号，或者如果您修改了gnss_comm来支持QZSS，就使用SYS_QZS
        // 假设您还未修改gnss_comm，我们可以暂时忽略它或返回0
         return 0; // 暂时不支持QZSS，因为它需要修改gnss_comm本身
    }
    else {
        return 0; // 无法识别的ID
    }

    // --- 第2步: 重新编码 - 使用gnss_comm的逻辑生成正确的、连续的ID ---
    if (svid == 0) return 0;

    switch (sys) {
        case SYS_GPS:
            return svid - MIN_PRN_GPS + 1;
        case SYS_GLO:
            return N_SAT_GPS + (svid - MIN_PRN_GLO + 1);
        case SYS_GAL:
            return N_SAT_GPS + N_SAT_GLO + (svid - MIN_PRN_GAL + 1);
        case SYS_BDS:
            return N_SAT_GPS + N_SAT_GLO + N_SAT_GAL + (svid - MIN_PRN_BDS + 1);
    }
    
    return 0; // 如果系统类型不匹配，返回无效ID
}

/**
 * @brief 计算由于错误的时间基准导致的伪距固定偏差。
 *
 * 这个函数计算了因错误地使用“GPS总秒数”而非“GPS周内秒”与
 * 各个星座自身的“周内秒”作差而引入的固定距离偏差。
 *
 * @param sys 卫星系统类型 (e.g., SYS_GPS, SYS_BDS)。
 * @param obs_week 当前观测的GPS周数，用于计算总的GPS时间。
 * @return double 需要从错误伪距中减去的修正偏差（单位：米）。
 */
double calculate_pseudorange_offset(uint32_t sys, uint32_t obs_week)
{
    const static double gpst0[] = {1980,1,6,0,0,0}; /* gps time reference */
    const static double gst0 [] = {1999,8,22,0,0,0}; /* galileo system time reference */
    const static double bdt0 [] = {2006,1,1,0,0,0}; /* beidou time reference */


    // C++11及以上版本保证静态局部变量是线程安全的
    static const gtime_t gps_epoch = epoch2time(gpst0);
    static const gtime_t bdt_epoch = epoch2time(bdt0);
    static const gtime_t gst_epoch = epoch2time(gst0);

    // 偏差主要由GPS总周数引起
    double base_offset_sec = static_cast<double>(obs_week) * WEEK_SECONDS;
    // 不同系统相对于GPS时间的额外固定偏差
    double system_time_diff_sec = 0.0;

    switch (sys)
    {
        case SYS_GPS:
            // GPS的偏差就是纯粹的周数偏差
            system_time_diff_sec = 0.0;
            break;
        
        case SYS_BDS:
            // 北斗时(BDT)与GPS时的起点不同，且有14秒的固定差异
            // time_diff(t1, t2) = t1 - t2
            system_time_diff_sec = time_diff(gps_epoch, bdt_epoch) + 14.0;
            break;

        case SYS_GAL:
            // 伽利略时(GST)与GPS时的起点不同
            system_time_diff_sec = time_diff(gps_epoch, gst_epoch);
            break;
            
        case SYS_GLO:
            // GLONASS使用UTC，较为复杂。一个简化的处理是假设其与GPS周对齐，
            // 但考虑到闰秒，其周内秒与GPS周内秒存在差异。
            // 幸运的是，安卓GnssClock中的fullBiasNanos已经将设备时间对齐到了GPS时间，
            // 所以GLONASS卫星的receivedSvTimeNanos实际上也是相对于GPS周的。
            // 因此，对于GLONASS，偏差与GPS相同。
            system_time_diff_sec = 0.0;
            break;

        // 如果您未来支持QZSS，它的时间系统与GPS相同
        case SYS_QZS:
            system_time_diff_sec = 0.0;
            break;
            
        default:
            return 0.0;
    }

    // 总偏差 = (GPS周数偏差 + 星座间时间系统偏差) * 光速
    return (base_offset_sec + system_time_diff_sec) * LIGHT_SPEED;
}

class SppProcessor
{
public:
    // 构造函数，当创建 SppProcessor 对象时被调用
    SppProcessor(ros::NodeHandle& nh)
    {
        // 初始化 ROS 订阅者
        // 订阅伪距测量值，话题名称与你的 rosbag 中的一致
        sub_range_meas_ = nh.subscribe("ublox_driver/range_meas", 10, &SppProcessor::rangeMeasCallback, this);
        // 订阅 GPS/Galileo/BeiDou 的星历
        sub_ephem_ = nh.subscribe("ublox_driver/ephem", 100, &SppProcessor::ephemCallback, this);
        // 订阅 GLONASS 的星历
        sub_glo_ephem_ = nh.subscribe("ublox_driver/glo_ephem", 100, &SppProcessor::gloEphemCallback, this);
        // 订阅电离层参数
        sub_iono_params_ = nh.subscribe("ublox_driver/iono_params", 10, &SppProcessor::ionoParamsCallback, this);

        // 初始化 ROS 发布者
        // 我们将计算出的 PVT (Position, Velocity, Time) 结果发布到这个话题
        pub_spp_soln_ = nh.advertise<GnssPVTSolnMsg>("gnss_comm/spp_solution", 10);
    }

    // 处理 GPS/Galileo/BeiDou 星历的回调函数
    void ephemCallback(const GnssEphemMsgConstPtr& msg)
    {
        // 使用 gnss_comm 库提供的 msg2ephem 函数，将 ROS 消息转换成内部 Ephem 结构体
        EphemPtr ephem = msg2ephem(msg);
        // 以卫星 ID 为 key，将最新的星历数据存储到我们的 map 中
        sat_ephems_[ephem->sat] = ephem;
    }

    // 处理 GLONASS 星历的回调函数
    void gloEphemCallback(const GnssGloEphemMsgConstPtr& msg)
    {
        // 同样，将 ROS 消息转换为内部 GloEphem 结构体
        GloEphemPtr glo_ephem = msg2glo_ephem(msg);
        // 存入 map
        sat_ephems_[glo_ephem->sat] = glo_ephem;
    }

    // 处理电离层参数的回调函数
    void ionoParamsCallback(const StampedFloat64ArrayConstPtr& msg)
    {
        // 直接将消息中的数据部分存储到我们的向量中
        iono_params_ = msg->data;
    }

    // 这是最核心的回调函数，处理伪距测量数据并进行 SPP 计算
    void rangeMeasCallback(const GnssMeasMsgConstPtr& msg)
    {
        // 步骤 1: 将 ROS 消息转换为内部的观测数据结构
        std::vector<ObsPtr> obs_vec = msg2meas(msg);
        if (obs_vec.empty())
        {
            return; // 如果没有观测数据，直接返回
        }

        // 步骤 2: 为每个观测数据找到对应的星历
        std::vector<ObsPtr> valid_obs;
        std::vector<EphemBasePtr> valid_ephems;

        // // 列出所有观测到的gnss卫星，，按星系排列
        // ROS_INFO("Observed satellites:");
        // ROS_INFO("PRN\tC/N0\tPSR");
        // ROS_INFO("---------------------------");
        // for (const auto& obs : obs_vec)
        // {
        //     ROS_INFO("%d\t%.1f\t%.1f",
        //              obs->sat, obs->CN0[0], obs->psr[0]);
        // }

        for (auto& obs : obs_vec)
        {
            // 检查我们是否已经接收到了这颗卫星的星历
            if (sat_ephems_.find(obs->sat) != sat_ephems_.end())
            {
                // --- 临时剔除GLONASS的逻辑 开始 ---
                
                const uint32_t sys = satsys(obs->sat, NULL);

                if (sys == SYS_GLO || sys == SYS_BDS || sys == 0 || sys == SYS_GAL)
                {
                    // 如果是GLONASS卫星，则直接跳过，不进行处理
                    continue; 
                }
                
                // --- 临时剔除GLONASS的逻辑 结束 ---

                // 如果不是GLONASS，就把这次观测和对应的星历分别存起来
                valid_obs.push_back(obs);
                valid_ephems.push_back(sat_ephems_.at(obs->sat));
            }
        }

        // 列出所有观测到的gnss卫星，，按星系排列
        ROS_INFO("Observed satellites with ephem");
        ROS_INFO("PRN\tC/N0\tPSR");
        ROS_INFO("---------------------------");
        for (const auto& obs : valid_obs)
        {
            ROS_INFO("%d\t%.1f\t%.1f\t%.5f",
                     obs->sat, obs->CN0[0], obs->psr[0], obs->freqs[0]);
        }

        // 至少需要4颗卫星才能进行定位
        if (valid_obs.size() < 4)
        {
            ROS_WARN("Not enough valid satellites for SPP calculation. Found %zu.", valid_obs.size());
            return;
        }

        // 步骤 3: 调用 gnss_spp 库中的函数进行计算
        
        // a) 计算位置 (伪距定位)
        // psr_pos 函数会返回一个7维的向量，前3个是 ECEF 坐标 (x, y, z)，
        // 后面4个是不同卫星系统的时间偏差
        Eigen::Matrix<double, 7, 1> pos_result = psr_pos(valid_obs, valid_ephems, iono_params_);
        Eigen::Vector3d receiver_pos_ecef = pos_result.head<3>();
        LOG(INFO) << "Calculated receiver ECEF position: " << receiver_pos_ecef.transpose();

        // 如果计算出的位置向量的模为0，说明计算失败
        if (receiver_pos_ecef.norm() < 1.0)
        {
            ROS_ERROR("SPP position calculation failed.");
            return;
        }
        
        // b) 计算速度 (多普勒测速)
        // dopp_vel 函数需要一个参考位置来进行计算，我们用刚刚算出的伪距定位结果
        Eigen::Matrix<double, 4, 1> vel_result = dopp_vel(valid_obs, valid_ephems, receiver_pos_ecef);
        Eigen::Vector3d receiver_vel_ecef = vel_result.head<3>();

        // 步骤 4: 将计算结果封装成 ROS 消息并发布

        // a) 将 ECEF 坐标转换为大地坐标 (纬度, 经度, 高度)
        Eigen::Vector3d receiver_pos_lla = ecef2geo(receiver_pos_ecef); // [lat, lon, alt] in degrees and meters
        // b) 将 ECEF 速度转换为 NED (东北天) 坐标系下的速度
        Eigen::Vector3d receiver_vel_ned = ecef2enu(receiver_pos_lla, receiver_vel_ecef); // This gives ENU, need to swap to get NED
        
        // 创建一个 PVT 消息对象
        GnssPVTSolnMsg pvt_msg;
        // 填充消息的各个字段
        pvt_msg.time.week = msg->meas[0].time.week; // 使用第一个观测值的时间作为整个历元的时间
        pvt_msg.time.tow = msg->meas[0].time.tow;
        pvt_msg.fix_type = 3; // 3 = 3D-fix
        pvt_msg.valid_fix = true;
        pvt_msg.num_sv = valid_obs.size();
        pvt_msg.latitude = receiver_pos_lla.x();
        pvt_msg.longitude = receiver_pos_lla.y();
        pvt_msg.altitude = receiver_pos_lla.z();
        // NED 和 ENU 的 E/N, U/D 轴是相反的
        pvt_msg.vel_n = receiver_vel_ned.y(); // N
        pvt_msg.vel_e = receiver_vel_ned.x(); // E
        pvt_msg.vel_d = -receiver_vel_ned.z(); // D

        // 发布消息
        pub_spp_soln_.publish(pvt_msg);

        ROS_INFO("SPP solution published. Lat: %.6f, Lon: %.6f, Alt: %.2f", 
                 pvt_msg.latitude, pvt_msg.longitude, pvt_msg.altitude);
    }

private:
    // ROS 订阅者和发布者
    ros::Subscriber sub_range_meas_;
    ros::Subscriber sub_ephem_;
    ros::Subscriber sub_glo_ephem_;
    ros::Subscriber sub_iono_params_;
    ros::Publisher pub_spp_soln_;

    // 用 map 来存储每颗卫星 (key: sat_id) 最新的星历数据 (value: EphemBasePtr)
    // EphemBasePtr 是一个智能指针，可以指向 Ephem (GPS等) 或 GloEphem (GLONASS)
    std::map<uint32_t, EphemBasePtr> sat_ephems_;
    // 用 vector 存储电离层参数
    std::vector<double> iono_params_;
};

int main(int argc, char **argv)
{
    // 标准的 ROS 节点初始化
    ros::init(argc, argv, "spp_processor_node");
    ros::NodeHandle nh;
    
    // 创建 SppProcessor 对象，构造函数会自动设置好所有的订阅和发布
    SppProcessor node(nh);
    
    // ros::spin() 会让节点一直运行，等待并处理回调函数
    ros::spin();
    
    return 0;
}