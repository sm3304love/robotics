#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>

double calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_box_motion_controller");
    ros::NodeHandle nh;

    ros::Publisher modelStatePub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    ros::Publisher closestBoxPub = nh.advertise<gazebo_msgs::ModelState>("/closest_boxes", 10);

    std::vector<std::string> modelNames = {"box1", "box2", "box3", "box4", "box5", "box6", "box7", "box8", "box9", "box10"};

    std::vector<geometry_msgs::Pose> startPoses;
    std::vector<geometry_msgs::Pose> targetPoses;
    std::vector<double> rates;
    std::vector<double> elapsedTimes;

    double minX = -5.0;  // x 위치의 최소값
    double maxX = 4.0;   // x 위치의 최대값
    double minY = -5.0;  // y 위치의 최소값
    double maxY = 4.0;   // y 위치의 최대값
    double minZ = 0.0;   // z 위치의 최소값
    double maxZ = 0.0;   // z 위치의 최대값

    double animationDuration = 15.0;  // 애니메이션의 총 시간 (초)

    double minDistanceThreshold = 0.5;  // 최소 거리 임계값

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> randX(minX, maxX);
    std::uniform_real_distribution<double> randY(minY, maxY);
    std::uniform_real_distribution<double> randZ(minZ, maxZ);
    std::uniform_real_distribution<double> randRate(0.5, 2.0);  // 주기 범위

    for (int i = 0; i < modelNames.size(); i++)
    {
        geometry_msgs::Pose startPose;
        startPose.position.x = randX(gen)+1;  // 시작 위치 x
        startPose.position.y = randY(gen)+1;  // 시작 위치 y
        startPose.position.z = randZ(gen);  // 시작 위치 z

        geometry_msgs::Pose targetPose;
        targetPose.position.x = randX(gen);  // 종료 위치 x
        targetPose.position.y = randY(gen);  // 종료 위치 y
        targetPose.position.z = randZ(gen);  // 종료 위치 z

        startPoses.push_back(startPose);
        targetPoses.push_back(targetPose);

        double rate = 100.0 / randRate(gen);  // 애니메이션의 주기 (Hz)
        rates.push_back(rate);

        elapsedTimes.push_back(0.0);
    }

    ros::Rate loopRate(100.0);  // 메인 루프 주기 (Hz)

    while (ros::ok())
    {
        for (int i = 0; i < modelNames.size(); i++)
        {
            double t = elapsedTimes[i] / animationDuration;  // 보간을 위한 시간 파라미터 (0.0 ~ 1.0)

            if (t >= 1.0)
            {
                // 종료 위치에 도달한 경우, 시작 위치와 종료 위치를 교환하여 다시 이동 시작
                std::swap(startPoses[i], targetPoses[i]);
                elapsedTimes[i] = 0.0;
            }

            // 선형 보간을 사용하여 시작 위치에서 종료 위치로 이동
            double x = startPoses[i].position.x + (targetPoses[i].position.x - startPoses[i].position.x) * t;
            double y = startPoses[i].position.y + (targetPoses[i].position.y - startPoses[i].position.y) * t;
            double z = startPoses[i].position.z + (targetPoses[i].position.z - startPoses[i].position.z) * t;

            gazebo_msgs::ModelState modelState;
            modelState.model_name = modelNames[i];
            modelState.pose.position.x = x;
            modelState.pose.position.y = y;
            modelState.pose.position.z = z;

            modelStatePub.publish(modelState);

            elapsedTimes[i] += 1.0 / rates[i];

            // 가까운 박스 찾기
            double closestDistance = std::numeric_limits<double>::max();
            int closestIndex = -1;

            for (int j = 0; j < modelNames.size(); j++)
            {
                if (i != j)
                {
                    double distance = calculateDistance(modelState.pose, modelState.pose);
                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        closestIndex = j;
                    }
                }
            }

            if (closestDistance < minDistanceThreshold && closestIndex != -1 && i != closestIndex)
            {
                // 가까운 두 개의 박스 정보 publish
                gazebo_msgs::ModelState closestBox1 = modelState;
                gazebo_msgs::ModelState closestBox2 = modelState;

                if (closestBox1.model_name != closestBox2.model_name)
                {
                    closestBoxPub.publish(closestBox1);
                    closestBoxPub.publish(closestBox2);

                }
            }
        }

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
    
}



