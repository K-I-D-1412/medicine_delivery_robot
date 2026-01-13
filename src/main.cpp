#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include "iostream"
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
#define pi 3.14159

ros::Publisher vel_pub; // ROS发布者，用于发布速度消息
geometry_msgs::Twist vel_msg;   // 速度消息

// 相机状态枚举
enum CameraState
{
    COMPUTER = 0,
    ZED,
    REALSENSE
};

// 机器人状态枚举
enum ROBOSTATE
{
	FORWARD=0,
	CHECK,
	RIGHT,
	LEFT,
	STAY,
	STOP,
	END
};

CameraState state = REALSENSE;  //本实验使用REALSENSE摄像头
ROBOSTATE flag = FORWARD;   // 当前机器人状态为前进
int check_times=0;  // 检查次数

// 函数：检测路障桶并返回其中心点
vector<Point> detectCones(Mat frame) {
    Mat hsv, mask ,mask1, mask2;
    vector<vector<Point>> contours;
    vector<Point> centers;

    
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 根据路障颜色的HSV范围创建掩膜
    inRange(hsv, Scalar(0, 150, 150), Scalar(10, 255, 255), mask1);// 红色高范围
    inRange(hsv, Scalar(170, 150, 150), Scalar(180, 255, 255), mask2);// 红色低范围
    bitwise_or(mask1, mask2, mask);

    // 寻找轮廓
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        // 过滤小面积噪声
        if (contourArea(contour) > 300) {
            Moments m = moments(contour);
            Point center(m.m10 / m.m00, m.m01 / m.m00); // 计算中心点
            centers.push_back(center);

            // 在图像上绘制轮廓和中心点
            drawContours(frame, vector<vector<Point>>{contour}, -1, Scalar(0, 255, 255), 5);
            circle(frame, center, 8, Scalar(0, 0, 0), -1);
        }
    }

    return centers;
}

void navigatePathWithSides(Mat frame, const vector<Point>& centers, double& angle, double& v) {
    // 情况一：如果没有检测到路障桶，无法计算路径
    if (centers.size() < 2 && flag != STOP && flag != END) {
        putText(frame, "ERROR: NO Cone", Point(10, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);
        return;
    }
	//printf("begin path");

    // 初始化左右侧路障桶的点集合
    vector<Point> leftCones, rightCones;
    int frameMiddle = frame.cols / 2;

    // 分类左右侧路障桶，以图像中心为分界线
    for (const auto& center : centers) {
        if (center.x < frameMiddle) {
            leftCones.push_back(center); 
        } else {
            rightCones.push_back(center);
        }
    }

    // 情况二：如果一侧没有路障桶，无法计算路径
    if (leftCones.empty() || rightCones.empty() && flag != END &&flag != STOP) {
        putText(frame, "ERROR: Only one side detected", Point(10, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);
        return;
    }
	
    // 计算左右侧的平均位置
    Point leftAvg(0, 0), rightAvg(0, 0);
    for (const auto& point : leftCones) {
        leftAvg.x += point.x;
        leftAvg.y += point.y;
    }
    
    leftAvg.x /= leftCones.size();
    leftAvg.y /= leftCones.size();

    for (const auto& point : rightCones) {
        rightAvg.x += point.x;
        rightAvg.y += point.y;
    }
    // 情况三：（转弯后），只要还有一侧路障桶，直行，直到两侧都没有路障桶
    if ((flag==STOP) && (leftCones.size() <= 1 || rightCones.size() <= 1)) {
    		putText(frame, "Straight", Point(50, 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 3);
    		//cout << "left" << leftCones.size() << "right" << rightCones.size() << endl;
    		v = 0.1;
    		angle = 0;
    		// 打印station
    		//cout << "go " << endl;
    		if(leftCones.size() <= 1 && rightCones.size() <= 1){
    			flag = END;
    			//cout << "end" << endl;
    		}
      		return;
      	}
    rightAvg.x /= rightCones.size();
    rightAvg.y /= rightCones.size();
    
    // 计算中线的中点
    Point midPoint((leftAvg.x + rightAvg.x) / 2, (leftAvg.y + rightAvg.y) / 2);

    // 定义机器人位置
    Point carPosition(frame.cols / 2, frame.rows);

    // 计算机器人当前位置到中点的向量
    Point direction = midPoint - carPosition;

    // 使用atan2计算角度（返回弧度）
    angle = atan2(direction.y, direction.x);

    // 将弧度转换为角度
    angle = angle * 180 / CV_PI + 90;


    // 识别模式
	if ((flag==FORWARD) && (leftCones.size() <= 4 || rightCones.size() <= 4)) {
    		flag = CHECK;
    		return;
	}
	if ((flag==LEFT || flag==RIGHT) && (leftCones.size() <= 2 && rightCones.size() <= 2)) {
  		vel_msg.linear.x=0;
        vel_msg.angular.z=0;
        vel_pub.publish(vel_msg);
        flag=STOP;	
  		return;
	}
	
}

Mat frame_msg;
void rcvCameraCallBack(const sensor_msgs::Image::ConstPtr & img) {
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}

//检测药片类型，返回药片数量
int pill_check(Mat img)
{
    int num=0;
    int num_blue = 0;
    int num_green = 0;

    // 裁剪图像
    int width = 0.6 * img.cols;
    Rect roi((img.cols - width) / 2, 0, width, 0.5 * img.rows);
    img = img(roi);

    // 高斯模糊
    GaussianBlur(img,img,Size(5,5),0);

    // 转换为HSV颜色空间
    Mat hsv_img;
    cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
    // imshow("ori", img);

    //直方图均衡化
    vector<cv::Mat> channels;
    split(hsv_img, channels);
    equalizeHist(channels[2], channels[2]);
    // channels[1]*=1.5;
    merge(channels, hsv_img);
    // cvtColor(hsv_img,img,COLOR_HSV2BGR);
    // imshow("equalize",img);

    // 蓝色和绿色的HSV范围
    Scalar low_blue(100, 70, 20);
    Scalar high_blue(130, 200, 200);

    Scalar low_green(40, 20, 20);
    Scalar high_green(80, 140, 200);

    Mat blue_mask;
    Mat green_mask;

    // 蓝色检测
    inRange(hsv_img, low_blue, high_blue, blue_mask);
    
    // 腐蚀和膨胀
    Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    morphologyEx(blue_mask, blue_mask, MORPH_CLOSE, kernel);
    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    // morphologyEx(blue_mask, blue_mask, MORPH_OPEN, kernel);

    //对蓝色掩膜图像进行连通区域分析，并统计面积大于50的蓝色区域的数量
    Mat areas_blue;
    int areas_blue_num = connectedComponents(blue_mask, areas_blue, 8, CV_32S);

    //统计各个连通区域的面积
    vector<int> area_size(areas_blue_num, 0);

    for (int row = 0; row < blue_mask.rows; row++)
    {
        for (int col = 0; col < blue_mask.cols; col++)
        {
            int area = areas_blue.at<int>(row, col);
            if (area > 0)
            {
                area_size[area]++;
            }
        }
    }

    //统计面积大于50的蓝色区域的数量
    for (int i = 1; i < areas_blue_num; i++)
    {
        if (area_size[i] > 50)
        {
            num_blue++;
        }
    }

    imshow("blue", blue_mask);

    // 绿色检测
    inRange(hsv_img, low_green, high_green, green_mask);

    // 腐蚀和膨胀
    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    morphologyEx(green_mask, green_mask, MORPH_OPEN, kernel);
    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    morphologyEx(green_mask, green_mask, MORPH_CLOSE, kernel);

    //对绿色掩膜图像进行连通区域分析，并统计面积大于50的绿色区域的数量
    Mat areas_green;
    int areas_green_num = connectedComponents(green_mask, areas_green, 8, CV_32S);

    //统计各个连通区域的面积
    vector<int> area_size_green(areas_green_num, 0);

    for (int row = 0; row < green_mask.rows; row++)
    {
        for (int col = 0; col < green_mask.cols; col++)
        {
            int area = areas_green.at<int>(row, col);
            if (area > 0)
            {
                area_size_green[area]++;
            }
        }
    }
    //统计面积大于50的绿色区域的数量
    for (int i = 1; i < areas_green_num; i++)
    {
        if (area_size_green[i] > 50)
        {
            num_green++;
        }
    }
    imshow("green", green_mask);

    //蓝色掩膜范围难以调整，当蓝色检测失误时，此方法可以保留一定容错率使识别结果正确
    if(abs(num_blue-6)<abs(num_green-8))
    {
        num=num_blue;
    }else
    {
        num=num_green;
    }

    // printf("pills'num=%d\n", num);
    return num;
}


int main(int argc, char ** argv ) {
    
    ros::init(argc,argv,"img_node");
    ros::NodeHandle n; 
    ros::Rate loop_rate(10);
    ros::Subscriber camera_sub;
    VideoCapture capture;
    vector<int> pill_num_list;
    int pill_num=0;
    waitKey(1000);
        if(state == COMPUTER)
    {
        capture.open(0); 
        if (!capture.isOpened())
        {
        printf("电脑摄像头没有正常打开\n");
        return 0;
        }
        waitKey(1000);
    }
    else if(state == ZED)
    {
        capture.open(4); 
        if (!capture.isOpened())
        {
        printf("ZED 摄像头没有正常打开\n");
        return 0;
        }
        waitKey(1000);
    }
    else if(state == REALSENSE)
    {
        camera_sub = n.subscribe("/camera/color/image_raw",1,rcvCameraCallBack);
        waitKey(1000);
    }
    Mat frame;
    vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    double angle= 0;
    double v=0.12;

    while (ros::ok()) {
        // 逐帧读取
        if(state == REALSENSE)
        {
            if(frame_msg.cols == 0)
            {
                printf("没有获取到 realsense 图像\n");
                ros::spinOnce();
                continue;
            }
            frame = frame_msg;
        }      
        else if(state == ZED)
        {
            capture.read(frame);
            if (frame.empty())
            {
                printf("没有获取到 ZED 图像\n");
                continue;
            }
            frame = frame(cv::Rect(0,0,frame.cols/2,frame.rows));//截取 zed 的左目图片
        }

        // 显示当前帧
        imshow("windowName", frame);
        //Mat ori_img=frame_msg;
	if(flag==FORWARD)
	{
		// 检测路障桶
         vector<Point> centers = detectCones(frame);
         // 路径规划
         navigatePathWithSides(frame, centers, angle, v);
         // 显示结果
        imshow("Cone Navigation Simulation", frame);
	}

    if(flag == FORWARD){ 
        vel_msg.linear.x=v;
        //printf("give a speed\n");
        vel_msg.angular.z=-0.01*angle;
        vel_pub.publish(vel_msg);
    }else if(flag==CHECK)
    {
        if(check_times<20)
        {
            pill_num_list.push_back(pill_check(frame_msg));
            check_times++;
        }else
        {
            //printf("check_times:%d\n",check_times);
            angle = 0;
            unordered_map<int,int> freq_map;
            for(int num : pill_num_list)
            {
                freq_map[num]++;
            }
            
            pill_num=pill_num_list[0];
            int max=0;
            for(const auto& pair: freq_map)
            {
                if(pair.second>max)
                {
                    max=pair.second;
                    pill_num=pair.first;
                }
            }
            // printf("pill_num:%d\n",pill_num);
        }
        
        //通过药片数量决定转向决策
        if(pill_num<7 && pill_num>0)
        {
            flag=LEFT;
            printf("flag=LEFT,num=%d\n",pill_num);
        }else if(pill_num>7)
        {
            flag=RIGHT;
            printf("flag=RIGHT,num=%d\n",pill_num);
        }

        //具体转向控制
        if(flag==RIGHT)
        {
            for(int i=0;i<600;i++)
            {
                vel_msg.linear.x=0.1;
                vel_msg.angular.z=-1;
                vel_pub.publish(vel_msg);
                waitKey(5);
            }
            flag = STAY;
        }
        else if(flag==LEFT)
        {
            for(int i=0;i<600;i++)
            {
                vel_msg.linear.x=0.1;
                vel_msg.angular.z=1;
                vel_pub.publish(vel_msg);
                waitKey(5);
            }
            flag = STAY;
        }
        
    }
    else if(flag == STAY){
        for(int i=0;i<200;i++)
            {
                vel_msg.linear.x=0;
                vel_msg.angular.z=0;
                vel_pub.publish(vel_msg);
                waitKey(5);
            }
            flag = STOP;
    }
else if(flag == STOP)
{
    // 检测路障桶
    vector<Point> centers = detectCones(frame);
    // 路径规划
    navigatePathWithSides(frame, centers, angle, v);
    //cout << "flag" << flag << endl;
        vel_msg.linear.x=v;
        vel_msg.angular.z=-0.01*angle;
        vel_pub.publish(vel_msg);
    // 显示结果
    imshow("Cone Navigation Simulation", frame);
}
else if(flag == END){
    vel_msg.linear.x=0;
        vel_msg.angular.z=0;
        //printf("final state\n");
        vel_pub.publish(vel_msg);
}
    ros::spinOnce(); // 处理回调函数
    waitKey(5);
    loop_rate.sleep();
}

    return 0;
}