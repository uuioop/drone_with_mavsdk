#include "ArucoTracker.hpp"
#include <sstream>

ArucoTrackerNode::ArucoTrackerNode(const rclcpp::NodeOptions & options)
	: Node("aruco_tracker", options)
{
	loadParameters();

	// TODO: params to adjust detector params
	// See: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
	auto detectorParams = cv::aruco::DetectorParameters();

	// See: https://docs.opencv.org/4.x/d1/d21/aruco__dictionary_8hpp.html
	auto dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

	_detector = std::make_unique<cv::aruco::ArucoDetector>(dictionary, detectorParams);

	auto qos = rclcpp::QoS(1).best_effort();

	_image_sub = create_subscription<sensor_msgs::msg::Image>(
			     "/image_raw", qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

	_camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
				   "/camera_info", qos, std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));

	// Publishers
	_image_pub = create_publisher<sensor_msgs::msg::Image>("/image_proc", qos);
	_target_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", qos);
}

void ArucoTrackerNode::loadParameters()
{
	// 声明ROS2参数 - 可在launch文件或命令行中配置
	declare_parameter<int>("aruco_id", 0);           // 目标ArUco标签ID，默认0
	declare_parameter<int>("dictionary", 2);         // DICT_4X4_250，OpenCV预定义字典
	declare_parameter<double>("marker_size", 0.5);   // 物理标签尺寸（米），用于位姿计算

	// 获取参数值
	get_parameter("aruco_id", _param_aruco_id);
	get_parameter("dictionary", _param_dictionary);
	get_parameter("marker_size", _param_marker_size);
	
	// 记录配置信息
	RCLCPP_INFO(get_logger(), "ArUco标签配置 - ID: %d, 字典类型: %d, 标签尺寸: %.2fm", 
	            _param_aruco_id, _param_dictionary, _param_marker_size);
}

void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		// 将ROS图像消息转换为OpenCV图像格式 (BGR8编码)
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		// 检测ArUco标签 - 获取标签ID和角点坐标
		std::vector<int> ids;                                    // 检测到的标签ID列表
		std::vector<std::vector<cv::Point2f>> corners;          // 对应的角点坐标
		_detector->detectMarkers(cv_ptr->image, corners, ids);  // 执行检测
		cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);  // 在图像上绘制检测结果

		// 确保相机标定参数已获取（用于位姿计算）
		if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {

			// 对检测到的角点进行去畸变处理
			std::vector<std::vector<cv::Point2f>> undistortedCorners;

			for (const auto& corner : corners) {
				std::vector<cv::Point2f> undistortedCorner;
				cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
				undistortedCorners.push_back(undistortedCorner);
			}

			// 处理每个检测到的标签
			for (size_t i = 0; i < ids.size(); i++) {
				// 只处理配置的目标标签ID
				if (ids[i] != _param_aruco_id) {
					continue;
				}

				// 根据物理标签尺寸构建3D物体坐标点
				float half_size = _param_marker_size / 2.0f;
				std::vector<cv::Point3f> objectPoints = {
					cv::Point3f(-half_size,  half_size, 0),  // 左上角 (相对于标签中心)
					cv::Point3f(half_size,  half_size, 0),   // 右上角
					cv::Point3f(half_size, -half_size, 0),   // 右下角
					cv::Point3f(-half_size, -half_size, 0)   // 左下角
				};

				// 使用PnP算法求解标签相对于相机的3D位姿
				cv::Vec3d rvec, tvec;  // 旋转向量和平移向量
				cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_IPPE);
				
				// 在图像上绘制坐标轴，便于可视化验证
				cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, _param_marker_size);

				// 将旋转向量转换为四元数（用于ROS位姿消息）
				cv::Mat rot_mat;
				cv::Rodrigues(rvec, rot_mat);  // 罗德里格斯公式：旋转向量->旋转矩阵
				cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();  // 旋转矩阵->四元数并归一化
				
				// 发布目标位姿信息供无人机控制系统使用
				geometry_msgs::msg::PoseStamped pose_msg;
				pose_msg.header.stamp = msg->header.stamp;      // 时间戳同步
				pose_msg.header.frame_id = "camera_frame";     // 参考坐标系：相机坐标系
				pose_msg.pose.position.x = tvec[0];            // X方向平移 (米)
				pose_msg.pose.position.y = tvec[1];            // Y方向平移 (米)
				pose_msg.pose.position.z = tvec[2];            // Z方向平移 (米)
				pose_msg.pose.orientation.x = quat.x;          // 四元数X分量
				pose_msg.pose.orientation.y = quat.y;          // 四元数Y分量
				pose_msg.pose.orientation.z = quat.z;          // 四元数Z分量
				pose_msg.pose.orientation.w = quat.w;          // 四元数W分量

				_target_pose_pub->publish(pose_msg);

				// 在图像上添加文字标注（位置信息）
				annotate_image(cv_ptr, tvec);
				
				// 重要：只发布第一个检测到的目标位姿，避免多目标干扰
				break;
			}

		} else {
			RCLCPP_ERROR(get_logger(), "相机标定参数缺失 - 无法计算3D位姿");
		}

		// 始终发布处理后的图像（即使未检测到标签）
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header;                                    // 保持时间戳一致
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;          // BGR8编码
		out_msg.image = cv_ptr->image;                                  // 处理后的图像
		_image_pub->publish(*out_msg.toImageMsg().get());               // 发布到/image_proc话题

	} catch (const cv_bridge::Exception& e) {
		RCLCPP_ERROR(get_logger(), "cv_bridge异常: %s", e.what());
	}
}

void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	// 从相机信息消息更新相机内参矩阵（深拷贝确保数据安全）
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // 3x3内参矩阵
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // 畸变系数向量

	// 记录相机矩阵信息用于调试验证
	RCLCPP_INFO(get_logger(), "相机内参矩阵已更新:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	
	// 记录关键内参：焦距和主点坐标
	RCLCPP_INFO(get_logger(), "相机内参: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx - X方向焦距
		    _camera_matrix.at<double>(1, 1), // fy - Y方向焦距
		    _camera_matrix.at<double>(0, 2), // cx - X方向主点
		    _camera_matrix.at<double>(1, 2)  // cy - Y方向主点
		   );

	// 验证焦距参数有效性（为零表示标定数据异常）
	if (_camera_matrix.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(get_logger(), "错误：焦距参数为零，相机标定数据异常!");

	} else {
		RCLCPP_INFO(get_logger(), "成功从/camera_info话题更新相机内参");

		// 获取相机内参后取消订阅，避免重复处理
		RCLCPP_INFO(get_logger(), "取消订阅相机信息话题，内参已获取完成");
		_camera_info_sub.reset();
	}
}

void ArucoTrackerNode::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
{
	// 在图像上添加目标位置标注信息
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "X: "  << target[0] << " Y: " << target[1]  << " Z: " << target[2];
	std::string text_xyz = stream.str();

	// 设置文字显示参数
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1.0;
	int thickness = 2;
	int baseline = 0;
	
	// 计算文字尺寸，确定显示位置（右下角）
	cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;
	cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
	
	// 在图像上绘制文字（黄色，便于观察）
	cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

/**
 * @brief 主函数 - ArUco标签跟踪节点入口
 * 
 * 节点功能：
 * - 初始化ROS2系统
 * - 创建ArUcoTrackerNode实例
 * - 启动节点事件循环
 * - 处理ROS2关闭清理
 */
int main(int argc, char** argv)
{
	// 初始化ROS2系统
	rclcpp::init(argc, argv);
	
	// 创建节点选项（可传递命令行参数）
	rclcpp::NodeOptions options;
	
	// 创建并运行ArUco跟踪节点
	rclcpp::spin(std::make_shared<ArucoTrackerNode>(options));
	
	// 关闭ROS2系统，清理资源
	rclcpp::shutdown();
	return 0;
}