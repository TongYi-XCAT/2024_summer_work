#include "../inc/mark.hpp"



float pingfang(float x);
bool fanwei(float x, float min, float max);


Mat thre(Mat frame)
{
    Mat frame_thre = imgSplitG(frame);
    threshold(frame_thre, frame_thre, 170, 255, THRESH_BINARY);
    Mat elem = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
    cv::erode(frame_thre, frame_thre, elem);
    cv::dilate(frame_thre, frame_thre, elem);
    cv::dilate(frame_thre, frame_thre, elem);
    cv::erode(frame_thre, frame_thre, elem);

    return frame_thre;
}

vector<armor::Armor> mark(Mat frame_thre, Mat frame)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    cv::findContours(frame_thre, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //cout << contours.size() << endl;
    vector<RotatedRect> ctrs_rec;
    for(int i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i]) > 0) //看来这条件不加也行
        {
            ctrs_rec.push_back(minAreaRect(contours[i]));
        }
    }
    //cout << ctrs_rec.size() << endl;

    
    vector<RotatedRect> light_rec;
    for(int i = 0; i < ctrs_rec.size(); i++)
    {
        //条件一：灯条长宽比在3～12之间
        if(ctrs_rec[i].size.height > ctrs_rec[i].size.width?
        3 < ctrs_rec[i].size.height/ctrs_rec[i].size.width && ctrs_rec[i].size.height/ctrs_rec[i].size.width < 12:
        3 < ctrs_rec[i].size.width/ctrs_rec[i].size.height && ctrs_rec[i].size.width/ctrs_rec[i].size.height < 12)
        {
            //条件二：灯条长边的角度<10度，则这是血条
            /* 这是通过RotatedRect的属性实现的，可以和条件一合并 */
            if(ctrs_rec[i].size.height > ctrs_rec[i].size.width?
            ctrs_rec[i].angle < 80 : ctrs_rec[i].angle > 10)
            {
                light_rec.push_back(ctrs_rec[i]);
                /*
                Point2f vertices[4];
                ctrs_rec[i].points(vertices);
                for(int i = 0; i < 4; i++)
                {
                    line(frame, vertices[i], vertices[(i+1)%4], Scalar(0, 255, 0), 2);
                }
                for(const auto& point: vertices)
                {
                    circle(frame, point, 3, Scalar(0,0,255), FILLED);
                }
                */
            }
        }
        
    }
    //cout << light_rec.size() << endl;

    vector<array<RotatedRect,2>> armor_light_rec;
    vector<armor::ArmorType> armor_type;
    //cout << armor_light_rec.size() << endl;
    for(int i = 0; i < int(light_rec.size())-1; i++)
    {
        for(int j = i+1; j < light_rec.size(); j++)
        {
            //条件三：判断灯条是否平行
            if(abs(light_rec[i].angle - light_rec[j].angle) < 4
            || abs(90 - (light_rec[i].angle + light_rec[j].angle)) < 4
            && (5 > light_rec[i].angle || light_rec[i].angle > 85))
            {
                
                //条件四前置：两灯条长 长度相差不大
                if(
                abs(light_rec[i].size.height > light_rec[i].size.width?
                light_rec[i].size.height : light_rec[i].size.width
                -
                light_rec[j].size.height > light_rec[j].size.width?
                light_rec[j].size.height : light_rec[j].size.width) < 100 //注意这里
                ){
                    /**/
                    //条件四：装甲板长宽平方比
                    if(fanwei
                    (2*(pingfang(light_rec[i].center.x - light_rec[j].center.x)
                    + pingfang(light_rec[i].center.y - light_rec[j].center.y))
                    /
                    (pingfang(
                    light_rec[i].size.height > light_rec[i].size.width?
                    light_rec[i].size.height : light_rec[i].size.width)
                    +
                    pingfang(
                    light_rec[j].size.height > light_rec[j].size.width?
                    light_rec[j].size.height : light_rec[j].size.width)),
                    4,8)
                    ){
                        array<RotatedRect, 2> temp;
                        temp[0] = light_rec[i];
                        temp[1] = light_rec[j];
                        armor_light_rec.push_back(temp);
                        armor_type.push_back(armor::SMALL);
                    }
                    else 
                    if(fanwei
                    (2*(pingfang(light_rec[i].center.x - light_rec[j].center.x)
                    + pingfang(light_rec[i].center.y - light_rec[j].center.y))
                    /
                    (pingfang(
                    light_rec[i].size.height > light_rec[i].size.width?
                    light_rec[i].size.height : light_rec[i].size.width)
                    +
                    pingfang(
                    light_rec[j].size.height > light_rec[j].size.width?
                    light_rec[j].size.height : light_rec[j].size.width)),
                    12,23)
                    )
                    {
                        array<RotatedRect, 2> temp;
                        temp[0] = light_rec[i];
                        temp[1] = light_rec[j];
                        armor_light_rec.push_back(temp);
                        armor_type.push_back(armor::LARGE);
                    }
                    
                    /*
                    array<RotatedRect, 2> temp;
                    temp[0] = light_rec[i];
                    temp[1] = light_rec[j];
                    armor_light_rec.push_back(temp);
                    */
                }
                
                /*
                array<RotatedRect, 2> temp;
                temp[0] = light_rec[i];
                temp[1] = light_rec[j];
                armor_light_rec.push_back(temp);
                */
            }
        }
    }
    //cout << armor_light_rec.size() << endl;
    
    /*
    for(int i = 0; i < armor_light_rec.size(); i++)
    {
        Point2f vertices[4];
        armor_light_rec[i][0].points(vertices);
        for(int j = 0; j < 4; j++)
        {
            line(frame, vertices[j], vertices[(j+1)%4], Scalar(0,255,0), 4);
        }
        armor_light_rec[i][1].points(vertices);
        for(int j = 0; j < 4; j++)
        {
            line(frame, vertices[j], vertices[(j+1)%4], Scalar(0,255,0), 4);
        }
        imshow("Frame_mark", frame);
        waitKey(0);
    }
    */
    
    //调整灯条顺序，存角点并调整顺序
    vector<array<Point2f,4>> armor_rec_vertices;
    for(auto combo: armor_light_rec)
    {
        if(combo[0].center.x > combo[1].center.x)
        {
            RotatedRect temp = combo[0];
            combo[0] = combo[1];
            combo[1] = temp;
        }
        /*
        for(int i = 0; i < armor_light_rec.size(); i++)
        {
            Point2f vertices[4];
            combo[0].points(vertices);
            for(int j = 0; j < 4; j++)
            {
                line(frame, vertices[j], vertices[(j+1)%4], Scalar(0,255,0), 4);
            }
            combo[1].points(vertices);
            for(int j = 0; j < 4; j++)
            {
                line(frame, vertices[j], vertices[(j+1)%4], Scalar(0,0,255), 4);
            }
            imshow("Frame_mark", frame);
            waitKey(0);
        }
        */
        array<Point2f,4> armor_vertices;
        Point2f left_light[4];
        combo[0].points(left_light);
        if(combo[0].angle > 45) //注意，这个条件可能在与之前的血条角度条件（条件二）联动时出现问题？
        {
            armor_vertices[0] = left_light[3];
            //circle(frame, armor_vertices[0], 3, Scalar(255,0,0), FILLED);
            //putText(frame, "3", armor_vertices[0], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            //imshow("Frame_mark", frame);
            //waitKey(0);
            armor_vertices[1] = left_light[0];
            //circle(frame, armor_vertices[1], 3, Scalar(255,0,0), FILLED);
            //putText(frame, "0", armor_vertices[1], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            //imshow("Frame_mark", frame);
            //waitKey(0);
        }
        else
        {
            armor_vertices[0] = left_light[0];
            //circle(frame, armor_vertices[0], 3, Scalar(255,0,0), FILLED);
            //putText(frame, "0", armor_vertices[0], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            //imshow("Frame_mark", frame);
            //waitKey(0);
            armor_vertices[1] = left_light[1];
            //circle(frame, armor_vertices[1], 3, Scalar(255,0,0), FILLED);
            //putText(frame, "1", armor_vertices[1], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            //imshow("Frame_mark", frame);
            //waitKey(0);
        }
        Point2f right_light[4];
        combo[1].points(right_light);
        if(combo[1].angle > 45) //注意，这个条件可能在与之前的血条角度条件（条件二）联动时出现问题
        {
            armor_vertices[2] = right_light[1];
            //circle(frame, armor_vertices[2], 3, Scalar(255,0,0), FILLED);
            //putText(frame, "1", armor_vertices[2], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            //imshow("Frame_mark", frame);
            //waitKey(0);
            armor_vertices[3] = right_light[2];
            //circle(frame, armor_vertices[3], 3, Scalar(255,0,0), FILLED);
            //putText(frame, "2", armor_vertices[3], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            //imshow("Frame_mark", frame);
            //waitKey(0);
        }
        else
        {
            armor_vertices[2] = right_light[2];
            //circle(frame, armor_vertices[2], 3, Scalar(255,0,0), FILLED);
            //putText(frame, "2", armor_vertices[2], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            //imshow("Frame_mark", frame);
            //waitKey(0);
            armor_vertices[3] = right_light[3];
            //circle(frame, armor_vertices[3], 3, Scalar(255,0,0), FILLED);
            //putText(frame, "3", armor_vertices[3], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            //imshow("Frame_mark", frame);
            //waitKey(0);
        }
        armor_rec_vertices.push_back(armor_vertices);
        
    }
    
    //可视化 存参返回
    vector<armor::Armor> armors;
    /*
    for(auto ver: armor_rec_vertices)
    {
        armor::Armor temp(ver);
        //line(frame, temp.left_light.bottom, temp.right_light.top, Scalar(0, 255, 0), 2);
        //line(frame, ver[0], ver[2], Scalar(0, 255, 0), 2);
        //imshow("Frame_mark", frame);
        //waitKey(0);
        //line(frame, temp.left_light.top, temp.right_light.bottom, Scalar(0, 255, 0), 2);
        //line(frame, ver[1], ver[3], Scalar(0, 255, 0), 2);
        //imshow("Frame_mark", frame);
        //waitKey(0);
        armors.push_back(temp);
    }
    */
    //必须armor_rec_vertices.size() == armor_type.size()
    //cout << (armor_rec_vertices.size() == armor_type.size()) << endl;
    for(int i = 0; i < armor_rec_vertices.size(); i++)
    {
        armor::Armor temp(armor_rec_vertices[i], armor_type[i]);
        armors.push_back(temp);
    }

    return armors;
}

Mat imgSplitG(Mat img)
{
    Mat Fsplit[3];
    split(img, Fsplit);
    img = Fsplit[1];
    return img;
}

float pingfang(float x)
{
    return x*x;
}

bool fanwei(float x, float min, float max)
{
    return min < x && x < max;
}