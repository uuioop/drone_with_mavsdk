#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief ArUco标签跟踪节点 - 用于视觉导航和精准降落
 * 
 * 该节点负责：
 * 1. 从相机图像中检测ArUco标签
 * 2. 计算标签相对于相机的3D位姿
 * 3. 发布目标位姿信息供无人机控制系统使用
 * 4. 提供可视化标注功能
 */
class ArucoTrackerNode : public rclcpp::Node
{
public:
	/**
	 * @brief 构造函数 - 初始化ArUco跟踪节点
	 * @param options ROS2节点选项
	 */
	explicit ArucoTrackerNode(const rclcpp::NodeOptions & options);

private:
	/**
	 * @brief 加载ROS2参数 - 包括ArUco字典类型、标签ID、标签尺寸等
	 */
	void loadParameters();

	/**
	 * @brief 图像回调函数 - 处理相机图像，检测ArUco标签并计算位姿
	 * @param msg 传感器图像消息
	 */
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
	
	/**
	 * @brief 相机信息回调函数 - 获取相机内参和畸变系数
	 * @param msg 相机信息消息，包含内参矩阵和畸变参数
	 */
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
	
	/**
	 * @brief 图像标注函数 - 在图像上绘制标签位置和坐标信息
	 * @param image OpenCV图像指针
	 * @param target 检测到的标签3D位置向量
	 */
	void annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target);

	// ROS2通信接口
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;      ///< 图像订阅器
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;  ///< 相机信息订阅器
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;        ///< 处理后的图像发布器
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_pub;  ///< 目标位姿发布器

	// OpenCV相关
	std::unique_ptr<cv::aruco::ArucoDetector> _detector;  ///< ArUco检测器
	cv::Mat _camera_matrix;    ///< 相机内参矩阵 (3x3)
	cv::Mat _dist_coeffs;      ///< 相机畸变系数

	// 参数配置
	int _param_aruco_id {};     ///< 目标ArUco标签ID
	int _param_dictionary {};   ///< ArUco字典类型 (默认DICT_4X4_250)
	double _param_marker_size {};  ///< 物理标签尺寸 (米)
};

