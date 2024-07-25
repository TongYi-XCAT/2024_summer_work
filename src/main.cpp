#include <iostream>
#include <fstream>
#include <string>
//计算程序运行速率
#include <chrono>

//传统图像处理
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//卡尔曼滤波所需的位姿和矩阵变换
#include <Eigen/Core>
#include <Eigen/Dense>

//用的很好，下次别用了，命名空间已经冲突了
//你可以打个vector<int>试试
using namespace std;
using namespace cv;
using namespace Eigen;

//图像处理
#include "../inc/mark.hpp"
//装甲板数字识别
#include "../inc/number_classifier.hpp"
//用于滤出敌车中心的卡尔曼
#include "../inc/kalman.hpp"

//把OpenCV的PnP解算得到的旋转向量转化成Eigen库的旋转向量
AngleAxisd toAA(vector<double> rvec);
//用于求两点间距离，实际是求三维向量之间的距离
double get_length(vector<double> tvec0, vector<double> tvec1);

//全局变量，装甲的尺寸
const float SMALL_ARMOR_WIDTH  = 0.135;
const float SMALL_ARMOR_HEIGHT = 0.055;
const float LARGE_ARMOR_WIDTH  = 0.225;
const float LARGE_ARMOR_HEIGHT = 0.055;
// x 垂直装甲板向内，从左下角开始顺时针
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
    { 0, +SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2},
    { 0, +SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
    { 0, -SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
    { 0, -SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2}
};
// x 垂直装甲板向内，从左下角开始顺时针
const std::vector<cv::Point3f> LARGE_ARMOR_POINTS = {
    { 0, +LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2 },
    { 0, +LARGE_ARMOR_WIDTH / 2, +LARGE_ARMOR_HEIGHT / 2 },
    { 0, -LARGE_ARMOR_WIDTH / 2, +LARGE_ARMOR_HEIGHT / 2 },
    { 0, -LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2 }
};
//相机内参
cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 2102.080562187802, 0, 689.2057889332623,
                                                0, 2094.0179120166754, 496.6622802275393,
                                                0, 0, 1);
//畸变矩阵
cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << -0.06478109387525666, 0.39036067923005396, 
-0.0042514793151166306, 0.008306749648029776, -1.6613800909405605);


//用于卡尔曼滤波的属性结构体
typedef struct {
    //相机坐标系下装甲板中心坐标
    vector<double> tvec;
    vector<double> rvec;
    //相机坐标系下y轴偏转角度
    double pitch_angle;
    //车心坐标
    vector<double> car_center;
    int id;
}kf;

//时间间隔，没用上
double dt = 0.01;
//状态矩阵初值
    Matrix<double, 6, 1> X0;
//状态关系矩阵
    Matrix<double, 6, 6> A;
//状态输入关系矩阵
    Matrix<double, 6, 6> B;
//状态输入矩阵
    Matrix<double, 6, 1> U;
//协方差矩阵初值
    Matrix<double, 6, 6> P0;
//状态协方差矩阵
    Matrix<double, 6, 6> Q;
//观测关系矩阵
    Matrix<double, 3, 6> H;
//观测初值
    Matrix<double, 3, 1> Z0;
//观测协方差矩阵
    Matrix<double, 3, 3> R;

//预测更新
array<Matrix<double, Dynamic, Dynamic>, 2> pdt;
//校正更新
array<Matrix<double, Dynamic, Dynamic>, 3> udt;

int main()
{
    //视频地址
    string path = "./video/armor.mp4";
    
    //视频容器
    VideoCapture vid_cap(path);
    //视频帧率和总帧数
    int fps, frame_count;
    //打开视频并输出帧率和总帧数
    if(!vid_cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
    }
    else
    {
        fps = vid_cap.get(CAP_PROP_FPS);
        cout << "Frames per second :" << fps << endl;

        frame_count = vid_cap.get(CAP_PROP_FRAME_COUNT);
        cout << "Frame count :" << frame_count << endl;
    }

    //已经播放的帧数
    int play_count = 0;
    //设置到某一帧
    vid_cap.set(CAP_PROP_POS_FRAMES, play_count);

    //创建一些固定尺寸的窗口
    //namedWindow("Frame", WINDOW_NORMAL);
    //resizeWindow("Frame", 600, 480);
    //namedWindow("Frame_thresold", WINDOW_NORMAL);
    //resizeWindow("Frame_thresold", 600, 480);
    //namedWindow("Frame_mark", WINDOW_NORMAL);
    //resizeWindow("Frame_mark", 600, 480);


    //卡尔曼赋初值
    X0 << 0,0,0,0,0,0;
    A << 1,0,0,dt,0,0,
         0,1,0,0,dt,0,
         0,0,1,0,0,dt,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;
    B << 0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0;
    U << 0,0,0,0,0,0;
    P0 << (10*10),0,0,0,0,0,
          0,(10*10),0,0,0,0,
          0,0,(10*10),0,0,0,
          0,0,0,(10*10),0,0,
          0,0,0,0,(10*10),0,
          0,0,0,0,0,(10*10);
    Q << (0.01*0.01),0,0,0,0,0,
         0,(0.01*0.01),0,0,0,0,
         0,0,(0.01*0.01),0,0,0,
         0,0,0,(0.01*0.01),0,0,
         0,0,0,0,(0.01*0.01),0,
         0,0,0,0,0,(0.01*0.01);
    H << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0;
    R << (1*1),0,0,
         0,(1*1),0,
         0,0,(1*1);

    //存放4号装甲板的要用于滤波的数据
    vector<kf> armor_4;
    //存放4号装甲板的滤波后的数据
    vector<vector<double>> armor_4_kf;
    //存放1号装甲板的滤波数据
    vector<kf> armor_1;

    //是否进行了卡尔曼第一次更新
    bool flag = false;

    //播放视频并处理，最好实现播放一帧、处理一帧、输出一帧结果
    while(vid_cap.isOpened())
    {
        //开启稳定时钟的计时
        chrono::steady_clock::time_point start = chrono::steady_clock::now();
        //存放一帧
        Mat frame;
        //记录是否得到一帧
        bool isSuccess = vid_cap.read(frame);

        //得到一帧后开始处理
        if(isSuccess == true)
        {
            //imshow("Frame", frame);
            
            //用mark.hpp的thre()二值化
            Mat frame_thre = thre(frame);
            //imshow("Frame_thresold", frame_thre);

            //筛选图像中的灯条，返回可能是装甲板的组合
            auto armors = mark(frame_thre, frame);

            //识别其中是否有数字，并只留下有数字的装甲板
            armor::NumberClassifier("./model/mlp.onnx", "./model/label.txt", 0.7, {"negative"}).ExtractNumbers(frame, armors);
            armor::NumberClassifier("./model/mlp.onnx", "./model/label.txt", 0.7, {"negative"}).Classify(armors);

            //对这帧的装甲板进行处理
            for(auto arm: armors)
            {
                //画叉
                //line(frame, arm.left_light.bottom, arm.right_light.top, Scalar(0, 255, 0), 2);
                //line(frame, arm.left_light.top, arm.right_light.bottom, Scalar(0, 255, 0), 2);
                
                //输出识别结果
                //cout << arm.classification_result << endl;
                //putText(frame, arm.classification_result, arm.left_light.top,
                //FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,255), 2);
                
                //cv::Mat tvec;
                //cv::Mat rvec;
                //准备PnP解算
                //存放位移向量
                vector<double> tvec;
                //存放旋转向量
                vector<double> rvec;
                //大小装甲板相似但不同处理
                if(arm.type == armor::SMALL)
                {
                    //PnP所需的特征点
                    std::vector<cv::Point2f> armor_vertices = {arm.left_light.bottom, arm.left_light.top,
                                                               arm.right_light.top, arm.right_light.bottom};
                    cv::solvePnP(SMALL_ARMOR_POINTS, armor_vertices, cameraMatrix, distCoeffs, rvec, tvec);
                    if(arm.classification_result[0] == '4')
                    {
                        //用于放进容器
                        kf temp;
                        //位移向量，也是相机坐标系下装甲板中心点的xyz坐标
                        temp.tvec = tvec;
                        //考虑不要了
                        temp.rvec = rvec;

                        //算出在相机坐标系下，绕y轴转过的角度
                        AngleAxisd rotation_vector = toAA(rvec);
                        Matrix3d rotation_matrix = Matrix3d::Identity();
                        rotation_matrix = rotation_vector.toRotationMatrix().cast<double>();
                        Vector3d euler_angles = rotation_matrix.eulerAngles(1, 2, 0);
                        //赋进结构体的角度
                        temp.pitch_angle = euler_angles(0);
                        
                        //赋图片id
                        temp.id = play_count;

                        //若这是第一个被识别出的4号装甲板
                        if(armor_4.size() == 0)
                        {
                            armor_4.push_back(temp);
                        }
                        //计算图片id相差，决定是否计算半径和车心坐标
                        else
                        {
                            if(temp.id - (armor_4.end()-1)->id == 1)
                            {
                                cout << (armor_4.end()-1)->id << " " << temp.id << endl;
                                //线速度
                                double line = get_length(temp.tvec, (armor_4.end()-1)->tvec);
                                //角速度
                                double angle = abs(temp.pitch_angle - (armor_4.end()-1)->pitch_angle);
                                //半径
                                double r = line / angle;
                                if(r > 1 || r < 0.08)
                                {
                                    r = 0.37;
                                }
                                //车心坐标结果
                                vector<double> result;
                                double x = temp.tvec[0] - cos(temp.pitch_angle) * r; 
                                result.push_back(x);
                                double y = temp.tvec[1];
                                result.push_back(y);
                                double z = temp.tvec[2] - sin(temp.pitch_angle) * r; 
                                result.push_back(z);

                                temp.car_center = result;
                                armor_4.push_back(temp);

                                //卡尔曼滤波，非常不严谨，每考虑到帧数间隔
                                if(!flag)
                                {
                                    flag = true;
                                    Z0 << result[0], result[1], result[2];
                                    pdt = KFpredict(X0, U, P0, A, B, Q);
                                    udt = KFupdate(pdt[0], pdt[1], Z0, H, R);
                                    
                                    //从Eigen的矩阵变到vector<double>
                                    vector<double> sender;
                                    for(int i = 0; i < 3; i++)
                                    {
                                        sender.push_back(udt[1](i,0));
                                    }
                                    armor_4_kf.push_back(sender);
                                }
                                else
                                {
                                    pdt = KFpredict(udt[1], U, udt[2], A, B, Q);
                                    Matrix<double, 3, 1> Z;
                                    Z << result[0], result[1], result[2];
                                    udt = KFupdate(pdt[0], pdt[1], Z, H, R);

                                    //从Eigen的矩阵变到vector<double>
                                    vector<double> sender;
                                    for(int i = 0; i < 3; i++)
                                    {
                                        sender.push_back(udt[1](i,0));
                                    }
                                    armor_4_kf.push_back(sender);
                                }
                            }
                            else
                            {
                                armor_4.push_back(temp);
                            }
                        }

                        
                    }
                    /*
                    string pnp_t = "armor center: (" + to_string(tvec[0]) + ", "+ to_string(tvec[1]) + ", " + to_string(tvec[2]) + ")";
                    string pnp_r = "armor angle(RPY): (" + to_string(rvec[0]) + ", "+ to_string(rvec[1]) + ", " + to_string(rvec[2]) + ")";
                    putText(frame, pnp_t, arm.right_light.bottom,
                    FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 2);
                    Point2f offset = {0, 14};
                    putText(frame, pnp_r, arm.right_light.bottom + offset,
                    FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 2);
                    */
                }
                if(arm.type == armor::LARGE)
                {
                    std::vector<cv::Point2f> armor_vertices = {arm.left_light.bottom, arm.left_light.top,
                                                               arm.right_light.top, arm.right_light.bottom};
                    cv::solvePnP(LARGE_ARMOR_POINTS, armor_vertices, cameraMatrix, distCoeffs, rvec, tvec);
                    if(arm.classification_result[0] == '1')
                    {
                        kf temp;
                        temp.tvec = tvec;
                        temp.rvec = rvec;
                        temp.id = play_count;
                        armor_1.push_back(temp);
                    }
                    /*
                    string pnp_t = "armor center: (" + to_string(tvec[0]) + ", "+ to_string(tvec[1]) + ", " + to_string(tvec[2]) + ")";
                    string pnp_r = "armor angle(RPY): (" + to_string(rvec[0]) + ", "+ to_string(rvec[1]) + ", " + to_string(rvec[2]) + ")";
                    putText(frame, pnp_t, arm.right_light.bottom,
                    FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 2);
                    Point2f offset = {0, 14};
                    putText(frame, pnp_r, arm.right_light.bottom + offset,
                    FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 2);
                    */
                }
            }

            //imshow("Frame_mark", frame);

            //处理完一帧
            play_count++;
        }
        /*
        string path4 = "./data/armor4_car_center.txt";
        ofstream fout4(path4, ios::out|ios::trunc);
        for(int i = 0; i < armor_4.size(); i++)
        {
            if(armor_4[i].car_center.size() != 0)
            {
                fout4 << armor_4[i].id << "         ";
                fout4 << armor_4[i].car_center[0] << " " << armor_4[i].car_center[1] << " " << armor_4[i].car_center[2] << endl;
            }
        }
        fout4.close();
        */
        string path4 = "./data/armor4_kf_car_center.txt";
        ofstream fout4(path4, ios::out|ios::trunc);
        for(int i = 0; i < armor_4_kf.size(); i++)
        {
            if(armor_4[i].car_center.size() != 0)
            {
                fout4 << armor_4_kf[i][0] << " " << armor_4_kf[i][1] << " " << armor_4_kf[i][2] << endl;
            }
        }
        fout4.close();
        //如果没得到一帧，要么是放完了，要么是断连了
        if(isSuccess == false)
        {
            cout << "Video camera is disconnected" << endl;
            cout << "Played progress :" 
            << to_string(play_count) << "/" << to_string(frame_count) << endl;
            break;
        }
        
        //计算处理一帧所需的时间
        chrono::steady_clock::time_point mark_end = chrono::steady_clock::now();
        chrono::steady_clock::duration mark_elapsed = mark_end - start;
        long long mark_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(mark_elapsed).count();
        /*
        //处理时间抽样调查
        if(play_count % 50 == 0)
        {
            cout << "mark_elapsed_ms = " << mark_elapsed_ms << "ms" << endl;
        }
        */
        /*
        //可以用于固定帧率，便于观察
        int key;
        if(1000/fps - mark_elapsed_ms <= 0)
        {
            //cout << "fps < 60" << endl;
            //key = waitKey(1);
            waitKey(0);
        }
        else
        {
            //key = waitKey(1000/fps - mark_elapsed_ms);
            //key = waitKey(1);
            waitKey(0);
        }
        
        //int key =waitKey(1000 - mark_elapsed_ms);
        //int key = waitKey(1);
        
        //固定帧率时，按q退出
        if(key == 'q')
        {
            cout << "q key is pressed by the user. Stopping the video" << endl;
            cout << "Played progress :" 
            << to_string(play_count) << "/" << to_string(frame_count) << endl;
            break;
        }
        */
        /*
        //本用于计算一个循环所用的时间，并抽样调查
        chrono::steady_clock::time_point loop_end = chrono::steady_clock::now();
        chrono::steady_clock::duration loop_elapsed = loop_end - start;
        long long loop_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(loop_elapsed).count();
        if(play_count % 50 == 0)
        {
            cout << "loop_elapsed_ms = " << loop_elapsed_ms << "ms" << endl;
        }
        */
    }
    



//要把后面的这堆处理放到前面去
    /*
    double dt = 0.01;
    Matrix<double, 6, 1> X0;
    X0 << 0,0,0,0,0,0;
    Matrix<double, 6, 6> A;
    A << 1,0,0,dt,0,0,
         0,1,0,0,dt,0,
         0,0,1,0,0,dt,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;
    Matrix<double, 6, 6> B;
    B << 0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0;
    Matrix<double, 6, 1> U;
    U << 0,0,0,0,0,0;
    Matrix<double, 6, 6> P0;
    P0 << (10*10),0,0,0,0,0,
          0,(10*10),0,0,0,0,
          0,0,(10*10),0,0,0,
          0,0,0,(10*10),0,0,
          0,0,0,0,(10*10),0,
          0,0,0,0,0,(10*10);
    Matrix<double, 6, 6> Q;
    Q << (0.01*0.01),0,0,0,0,0,
         0,(0.01*0.01),0,0,0,0,
         0,0,(0.01*0.01),0,0,0,
         0,0,0,(0.01*0.01),0,0,
         0,0,0,0,(0.01*0.01),0,
         0,0,0,0,0,(0.01*0.01);
 
    Matrix<double, 3, 6> H;
    H << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0;
    Matrix<double, 3, 1> Z0;
    Z0 << car_centers[0][0], car_centers[0][1], car_centers[0][2];
    Matrix<double, 3, 3> R;
    R << (1*1),0,0,
         0,(1*1),0,
         0,0,(1*1);
    //cout << X0 << endl << A << endl << B << endl << U << endl << P0 << endl << Q << endl << H << endl << Z0 << endl << R << endl;
    vector<vector<double>> KF_car_centers;
    
    array<Matrix<double, Dynamic, Dynamic>, 2> pdt;
    pdt = KFpredict(X0, U, P0, A, B, Q);
    
    array<Matrix<double, Dynamic, Dynamic>, 3> udt;
    udt = KFupdate(pdt[0], pdt[1], Z0, H, R);
    
    vector<double> sender;
    for(int i = 0; i < 3; i++)
    {
        sender.push_back(udt[1](i,0));
    }
    
    KF_car_centers.push_back(sender);
    for(auto dat = car_centers.begin()+1; dat != car_centers.end(); dat++)
    {
        pdt = KFpredict(udt[1], U, udt[2], A, B, Q);
        Matrix<double, 3, 1> Z;
        Z << dat->at(0), dat->at(1), dat->at(2);
        udt = KFupdate(pdt[0], pdt[1], Z, H, R);
        for(int i = 0; i < 3; i++)
        {
            sender[i] = udt[1](i,0);
        }
        KF_car_centers.push_back(sender);
    }
    
    string path4 = "./data/armor4_kf_car_center.txt";
    ofstream fout4(path4, ios::out|ios::trunc);
    for(int i = 0; i < KF_car_centers.size(); i++)
    {
        fout4 << KF_car_centers[i][0] << " " << KF_car_centers[i][1] << " " << KF_car_centers[i][2] << endl;
    }
    fout4.close();
    */
    /*
    string path4 = "./data/armor4.txt";
    ofstream fout4(path4, ios::out|ios::trunc);
    for(int i = 0; i < armor_4.size(); i++)
    {
        fout4 << armor_4[i].tvec[0] << " " << armor_4[i].tvec[1] << " " << armor_4[i].tvec[2] << " ";
        fout4 << armor_4[i].rvec[0] << " " << armor_4[i].rvec[1] << " " << armor_4[i].rvec[2] << " ";
        fout4 << armor_4[i].id << endl;
    }
    fout4.close();
    string path1 = "./data/armor1.txt";
    ofstream fout1(path1, ios::out|ios::trunc);
    for(int i = 0; i < armor_1.size(); i++)
    {
        fout1 << armor_1[i].tvec[0] << " " << armor_1[i].tvec[1] << " " << armor_1[i].tvec[2] << " ";
        fout1 << armor_1[i].rvec[0] << " " << armor_1[i].rvec[1] << " " << armor_1[i].rvec[2] << " ";
        fout1 << armor_1[i].id << endl;
    }
    fout1.close();
    */

    vid_cap.release();
    cv::destroyAllWindows();
    return 0;
}

AngleAxisd toAA(vector<double> rvec)
{
    double len = sqrt(rvec[0]*rvec[0]+rvec[1]*rvec[1]+rvec[2]*rvec[2]);
    AngleAxisd re(len, Vector3d(rvec[0]/len, rvec[1]/len, rvec[2]/len));
    return re;
}

double get_length(vector<double> tvec0, vector<double> tvec1)
{
    double len = (tvec1[0]-tvec0[0])*(tvec1[0]-tvec0[0])
               + (tvec1[2]-tvec0[2])*(tvec1[2]-tvec0[2]);
    len = sqrt(len);
    return len;
}