#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <vector>
#include <ctime>
#include <cstdio>
#include <limits>
#include <sstream>
#include <string>
#include <random>

#define RAD2DEG(x) ((x)*180./M_PI)
#define MAX_iteration 30
/*収束判定の閾値*/
#define EPS  0.001
#define delta 1.0e-7
/*学習率*/
#define learning_rate 1.2
struct Point {
    float x, y;
};

/*ロボットの自己位置*/
struct Pose{
    double x, y, theta;
};
int count = 0;
float distanceSquared = 0.0f;
float Error = std::numeric_limits<float>::max();
float initialError;
//マップのサイズと解像度
unsigned int size_x = 100; 
unsigned int size_y = 100;
float resolution = 0.1; //1セル=0.1メートル
float origin_x = 0.0;
float origin_y = 0.0;
bool initialized = false;
//占有グリッドマップ(-1: 未知, 0: 空き, 100: 占有)
std::vector<int> grid(size_x * size_y, -1);
float grid_size;
float alpha = 1;
float beta = 1 * M_PI /180;
float z_max = 10;
FILE* gnuplot_pipe = nullptr;
float preError = 0.0f;
size_t m_batch_size = 10;
std::vector<Point> target;
std::vector<Point> global_map;//グローバルマップを蓄積する
Pose current_pose = {0.0, 0.0, 0.0};  // ロボットの初期自己位置をグローバル変数として宣言


void plot(const std::vector<Point>& global_map) {
    if (!gnuplot_pipe) return;

    fprintf(gnuplot_pipe, "set size ratio 1\n");
    fprintf(gnuplot_pipe, "set xrange [-10:10]\n");
    fprintf(gnuplot_pipe, "set yrange [-10:10]\n");
    
    // プロットのためのデータを送信
    fprintf(gnuplot_pipe, "plot '-' with points pointtype 7 pointsize 0.5 lc rgb 'black' title 'MAP'\n");
    //fprintf(gnuplot_pipe, "'-' with points pointtype 7 pointsize 0.5 lc rgb 'black' title 'map'\n");

    // // レーザースキャンの点群をプロット
    // for (const auto& point : source) {
    //     fprintf(gnuplot_pipe, "%f %f\n", point.x, point.y);
    // }
    // fprintf(gnuplot_pipe, "e\n");

    // ファイルからの点群をプロット
    for (const auto& point : global_map) {
        fprintf(gnuplot_pipe, "%f %f\n", point.x, point.y);
    }
    fprintf(gnuplot_pipe, "e\n");

    // fprintf(gnuplot_pipe, "set arrow from %f,%f to %f,%f lc rgb 'red' lw 2\n", 
    //     current_pose.x, current_pose.y, 
    //     current_pose.x + cos(current_pose.theta) * 2, current_pose.y + sin(current_pose.theta) * 2);

    // fprintf(gnuplot_pipe, "e\n");

    //     fprintf(gnuplot_pipe, "set arrow from %f,%f to %f,%f lc rgb 'green' lw 2\n", 
    //     current_pose.x, current_pose.y, 
    //     current_pose.x - sin(current_pose.theta) * 2, current_pose.y + cos(current_pose.theta) * 2);



    fflush(gnuplot_pipe);
}

Point calculate_average(const std::vector<Point>& points) {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    for (const auto& point : points) {
        sum_x += point.x;
        sum_y += point.y;
    }
    float avg_x = sum_x / points.size();
    float avg_y = sum_y / points.size();
    return {avg_x, avg_y};
}

float distance(const Point& points, const Point& point){
    return sqrt((points.x - point.x) * (points.x - point.x) + (points.y - point.y) * (points.y - point.y));

}

std::array<std::array<float, 3>, 3> make_transformation_matrix(float tx, float ty, float theta) {
    return {{
        {std::cos(theta), -std::sin(theta), tx},
        {std::sin(theta), std::cos(theta), ty},
        {0.0f, 0.0f, 1.0f}
    }};
}

int findClosestPoint(const Point& point, const std::vector<Point>& target){
    int Index = -1;
    float minDist = std::numeric_limits<float>::max();
    for(size_t i = 0; i < target.size(); ++i){
        float dist = distance(target[i], point);
        if(dist < minDist){
            minDist = dist;
            Index = i;
        }
    }
    return Index;

}

std::vector<Point> transformpoints(const std::vector<Point>& points, float dx, float dy, double theta) {
    std::vector<Point> moved_points;
    auto transformation_matrix = make_transformation_matrix(dx, dy, theta);
    for (const auto& point : points) {
        float new_x = transformation_matrix[0][0] * point.x + transformation_matrix[0][1] * point.y + transformation_matrix[0][2];
        float new_y = transformation_matrix[1][0] * point.x + transformation_matrix[1][1] * point.y + transformation_matrix[1][2];
        moved_points.push_back({new_x, new_y});
    }
    return moved_points;
}

float diffx(Point Target, Point SOurce){
    float fx_delta = (Target.x - (SOurce.x + delta)) * (Target.x - (SOurce.x + delta)) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

float diffy(Point Target, Point SOurce){
    float fx_delta = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - (SOurce.y + delta)) * (Target.y - (SOurce.y + delta));
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

float difftheta(Point Target, Point SOurce){
    float fx_delta = (Target.x - ((SOurce.x)* cos(delta*M_PI/180)-(SOurce.y)* sin(delta*M_PI/180)))* (Target.x - ((SOurce.x)* cos(delta*M_PI/180)-(SOurce.y)* sin(delta*M_PI/180))) + (Target.y - ((SOurce.x) * (sin(delta*M_PI/180)) + (SOurce.y) * cos(delta*M_PI/180))) * (Target.y - ((SOurce.x) * (sin(delta*M_PI/180)) + (SOurce.y) * cos(delta*M_PI/180)));
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

void shuffle_data(std::vector<Point>& points) {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(points.begin(), points.end(), g);
}

std::vector<Point> createBatch(std::vector<Point>& source, size_t& m_current_offset, size_t m_batch_size) {
    std::vector<Point> batch;
    auto target_offset = m_current_offset + m_batch_size;

    while (target_offset >= source.size()) {
        while (m_current_offset < source.size()) {
            batch.push_back(source[m_current_offset++]);
        }
        shuffle_data(source);
        m_current_offset = 0;
        target_offset = target_offset - source.size();
    }
    while (m_current_offset < target_offset) {
        batch.push_back(source[m_current_offset++]);
    }

    return batch;
}


std::vector<Point> icp_scan_matching(std::vector<Point>& Source){//, const std::vector<Point>& target)
   //std::vector<Point> transformed_source = Source;
   auto start_time = std::chrono::high_resolution_clock::now();
   size_t m_current_offset = 0;
   shuffle_data(Source);
   float previous_error_sum = std::numeric_limits<float>::max(); // 前回の誤差を最大値で初期化
   for(int iter = 0; iter <= MAX_iteration; ++iter){
    auto batch = createBatch(Source, m_current_offset, m_batch_size);
    std::vector<Point> target_closest;
    float error_sum = 0;
    double gradDx = 0;
    double gradDy = 0;
    double gradTheta = 0;
    float dx = 0;
    float dy = 0;
    float dth = 0;
    float dtheta = 0;
    for(const auto& Source : batch){
         int index = findClosestPoint(Source,target);
         target_closest.push_back(target[index]);
         Point error = {target[index].x - Source.x, target[index].y - Source.y};
         Point Target = {target[index].x, target[index].y};
         //Point SOurce = {Source.x, Source.y};
         error_sum += error.x * error.x + error.y * error.y;
         gradDx += diffx(Target, Source);
         gradDy += diffy(Target, Source);
         gradTheta += difftheta(Target, Source);
    }

    /*収束条件のチェック*/
if(std::abs(previous_error_sum - error_sum) < EPS){
std::cout << "Converged after " << iter << "iterations." << std::endl;
break;
}

previous_error_sum = error_sum;//前回の誤差を更新
    int num_points =batch.size();

    dx = (-gradDx / num_points) * learning_rate;
    dy = (-gradDy / num_points) * learning_rate;
    dth = (-gradTheta / num_points) * learning_rate;


     for(auto& Source : Source){
    float x_new = Source.x * cos(dth) - Source.y * sin(dth);
    float y_new = Source.x * sin(dth) + Source.y * cos(dth);
    Source.x = x_new + dx;
    Source.y = y_new + dy;
}

//ロボットの自己位置を更新
    current_pose.x += dx;
    current_pose.y += dy;
    current_pose.theta += dth;
   }

   auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "ICP algorithm completed in " << duration.count() << " milliseconds." << std::endl;
   //変換された点群を返す
   return Source;
}


Point transformPoint(const Point& point, const Pose& pose){
    Point transformed_point;
    //回転行列と平行移動を使ってロボット座標系から世界座標系に変換
    transformed_point.x = pose.x + point.x * cos(pose.theta) - point.y * sin(pose.theta);
    transformed_point.y = pose.y + point.x * sin(pose.theta) + point.y * cos(pose.theta);
    return transformed_point;
}

/*LiDARのスキャンデータ(点群)を蓄積する*/
void accumulateMap(const std::vector<Point>& scan_points){
    for(const auto& point : scan_points) {
        // ICP 各点をロボットの自己位置を使って世界座標に変換
        global_map.push_back(point);
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->ranges.size();
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    std::vector<Point> points;

    for (int i = 0; i < count; i++) {
        float angle = scan->angle_min + scan->angle_increment * i;
        float range = scan->ranges[i];

        if (range >= scan->range_min && range <= scan->range_max) {
            float x = range * cos(angle);
            float y = range * sin(angle);

            points.push_back({x, y});
            //ROS_INFO("%.2f \t\t %.3f \t\t %.3f \t %.3f", RAD2DEG(angle), range, x, y);
        }
    }

    //スキャンデータをロボとの現在の位置に基づいて変換する
    std::vector<Point> transformed_points;
    for (const auto& point : points) {
        transformed_points.push_back(transformPoint(point, current_pose));
    }

    target = global_map;
    //最初のスキャンデータを使って初期の地図を作成
    if(global_map.empty()){
        global_map = transformed_points; //初期の地図として保存

    }else{
        //ICPによるスキャンデータのマッチングと地図の補正
        std::vector<Point> transformed_source = icp_scan_matching(transformed_points);

        //ICP決kの点群を世界座標に変換し、global_mapに蓄積
        accumulateMap(transformed_source);
    }

    plot(global_map);

    //std::vector<Point> file_points = readPointsFromFile("/home/ryosuke/catkin_ws/src/research/scan.txt
//  icp_scan_matching(points);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "LiDAR_node_SGD");
    ros::NodeHandle n;

    gnuplot_pipe = popen("gnuplot -persist", "w");
    if (!gnuplot_pipe) {
        ROS_ERROR("Could not open pipe to gnuplot");
        return 1;
    }

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);

    ros::spin();

  pclose(gnuplot_pipe);
    return 0;
}
