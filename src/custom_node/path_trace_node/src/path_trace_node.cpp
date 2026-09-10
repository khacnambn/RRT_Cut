#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <fstream>

ros::Publisher path_pub;
ros::Publisher nav_goal_pub;
nav_msgs::Path path;
double total_distance = 0.0; // Biến lưu tổng quãng đường
geometry_msgs::PoseStamped last_pose; // Lưu vị trí trước đó
bool is_first_pose = true; // Để kiểm tra lần đầu tiên
bool goal_received = false; // Cờ để theo dõi khi goal được nhận
bool goal_reached = false; // Cờ để theo dõi trạng thái của mục tiêu
bool data_saved = false; // Cờ để theo dõi khi dữ liệu đã được lưu
double move_start_time = 0.0; // Thời gian bắt đầu di chuyển
int total_points = 0;
uint32_t previous_seq = 0;
double total_turning_angle_sq = 0.0;   // ∑ θ^2


// Hàm tính khoảng cách giữa 2 điểm
double calculateDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Do muot
double computeTurningAngle(
    const geometry_msgs::PoseStamped& p1,
    const geometry_msgs::PoseStamped& p2,
    const geometry_msgs::PoseStamped& p3)
{
    double v1x = p2.pose.position.x - p1.pose.position.x;
    double v1y = p2.pose.position.y - p1.pose.position.y;
    double v2x = p3.pose.position.x - p2.pose.position.x;
    double v2y = p3.pose.position.y - p2.pose.position.y;

    double norm1 = std::sqrt(v1x * v1x + v1y * v1y);
    double norm2 = std::sqrt(v2x * v2x + v2y * v2y);

    if (norm1 < 1e-6 || norm2 < 1e-6)
        return 0.0;

    double dot = v1x * v2x + v1y * v2y;
    double cos_theta = dot / (norm1 * norm2);

    // Clamp để tránh lỗi số học
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));

    return std::acos(cos_theta);
}


// Hàm callback cho odometry để vẽ path và tính khoảng cách di chuyển
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!goal_received || goal_reached) return;
 // Chỉ bắt đầu khi đã chọn goal

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    // Nếu không phải là điểm đầu tiên, tính khoảng cách từ điểm trước đến điểm hiện tại
    if (!is_first_pose)
    {
        double dist = calculateDistance(last_pose, pose);
        total_distance += dist; // Cộng vào tổng quãng đường
    }
    else
    {
        is_first_pose = false; // Lần đầu tiên nhận vị trí
    }

    // Cập nhật vị trí cuối cùng
    last_pose = pose;
    path.poses.push_back(pose);
    // Tính turning angle khi có ít nhất 3 điểm
    int n = path.poses.size();
    if (n >= 3)
    {
        double theta = computeTurningAngle(
            path.poses[n - 3],
            path.poses[n - 2],
            path.poses[n - 1]
        );

        total_turning_angle_sq += theta * theta;
    }

    path.header.stamp = ros::Time::now();

     // Xuất path ra publisher
    path_pub.publish(path);

    // Xuất tổng quãng đường di chuyển
    // ROS_INFO("Total distance travelled: %f meters", total_distance);
}

// Callback để kiểm tra trạng thái của mục tiêu
void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    if (!msg->status_list.empty())
    {
        // Kiểm tra nếu trạng thái là SUCCEEDED (đã đạt mục tiêu)
        for (const auto& status : msg->status_list)
        {
            if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED && !data_saved)
            {
                goal_reached = true;
                double move_duration = ros::Time::now().toSec() - move_start_time; // Tính thời gian di chuyển
                
                
                // Lưu thông tin vào tệp YAML
                YAML::Node data;
                // Normalize smoothness theo chiều dài path
                double smoothness_norm = 0.0;
                if (total_distance > 1e-6)
                    smoothness_norm = total_turning_angle_sq / total_distance;

                data["time(s)"] = move_duration;
                data["cost(m)"] = total_distance;
                data["nodes"] = total_points;
                data["smoothness_turning_angle"] = total_turning_angle_sq;
                data["smoothness_norm"] = smoothness_norm;


                std::ofstream fout("/home/roab_lab/ros_motion_planning-master/src/custom_node/logdata/maze_11_data.yaml");
                fout << data; // Ghi dữ liệu vào tệp YAML
                fout.close();

                data_saved = true; // Đánh dấu rằng dữ liệu đã được lưu
                ROS_INFO("Goal reached! File saved");
                break;
            }
        }
    }
}

// Callback để nhận goal từ RViz
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    ROS_INFO("New goal received!");

    // Reset path và các biến trạng thái khi nhận goal mới
    goal_received = true;  // Đặt cờ khi goal được nhận
    goal_reached = false;  // Reset cờ mục tiêu đạt được
    data_saved = false;    // Reset cờ lưu dữ liệu
    is_first_pose = true;  // Reset trạng thái để bắt đầu tính toán từ đầu
    total_distance = 0.0;  // Reset tổng quãng đường
    total_turning_angle_sq = 0.0; // Do muot
    path.poses.clear();    // Xóa các điểm cũ trong path
    path.header.frame_id = "odom"; // Đảm bảo frame_id đúng cho path
    path.header.stamp = ros::Time::now(); // Reset thời gian cho path
    move_start_time = ros::Time::now().toSec(); // Ghi lại thời gian bắt đầu di chuyển

    // Xuất thông báo để biết path đã được reset
    ROS_INFO("Path and total distance reset for new goal.");
}

// Hàm để đặt một điểm NavGoal từ tọa độ x, y
void setNavGoal(double x, double y)
{
    geometry_msgs::PoseStamped nav_goal;
    nav_goal.header.frame_id = "map"; // Frame_id phù hợp
    nav_goal.header.stamp = ros::Time::now();
    nav_goal.pose.position.x = x;
    nav_goal.pose.position.y = y;
    nav_goal.pose.position.z = 0.0;
    nav_goal.pose.orientation.x = 0.0;
    nav_goal.pose.orientation.y = 0.0;
    nav_goal.pose.orientation.z = -0.7080808546550887;
    nav_goal.pose.orientation.w = 0.7061313640328682;

    // Gửi NavGoal tới topic /move_base_simple/goal
    nav_goal_pub.publish(nav_goal);

    ROS_INFO("NavGoal set at (x: %f, y: %f)", x, y);
}

// Hàm callback để xử lý thông báo từ topic /tree
void treeCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    // Lấy seq của thông điệp mới
    uint32_t current_seq = msg->header.seq;

    // Chỉ cộng dồn nếu seq của thông điệp mới lớn hơn seq trước đó
    if (current_seq > previous_seq) {
        // Cộng dồn số điểm vào tổng
        total_points += msg->points.size();

        // Cập nhật seq trước đó
        previous_seq = current_seq;

        // In ra tổng số điểm
        // ROS_INFO("Total points accumulated in tree: %d", total_points);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_trace_node");
    ros::NodeHandle nh;

    // Khởi tạo path publisher
    path_pub = nh.advertise<nav_msgs::Path>("robot_path", 10);
    
    // Khởi tạo NavGoal publisher
    nav_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    // ROS_INFO("Goal published to /move_base_simple/goal");
    
    // Đăng ký callback cho odometry, trạng thái của mục tiêu và goal từ RViz
    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
    ros::Subscriber goal_status_sub = nh.subscribe("move_base/status", 10, goalStatusCallback);
    ros::Subscriber goal_sub = nh.subscribe("move_base_simple/goal", 10, goalCallback); // Lắng nghe khi goal được chọn từ RViz

    //expand
    ros::Subscriber tree_sub = nh.subscribe("move_base/SamplePlanner/tree", 10, treeCallback);

    path.header.frame_id = "odom";  // Đặt frame_id phù hợp

    try {
        YAML::Node config = YAML::LoadFile("/home/roab_lab/ros_motion_planning-master/src/user_config/goal_config.yaml");
        double x = config["goal"]["x"].as<double>();
        double y = config["goal"]["y"].as<double>();
        setNavGoal(0.0, 0.0);
        ros::Duration(1.0).sleep();
        setNavGoal(x, y);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load goal.yaml: %s", e.what());
    }
    
    ros::spin();
    return 0;
}