#pragma once

#include <iostream>

namespace armor
{
    struct light
    {
        cv::Point2f top;
        cv::Point2f bottom;
    };
    
    enum ArmorType
    {
        SMALL,
        LARGE
    };
    
    class Armor
    {
    friend class NumberClassifier;

    private:
        double classification_confidence;
        cv::Mat number_image;
        std::string number;
        

    public:
        ArmorType type;
        light left_light;
        light right_light;
        std::string classification_result;
        Armor(std::array<cv::Point2f,4> armor_vertices, ArmorType armor_type)
        {
            left_light.bottom = armor_vertices[0];
            left_light.top = armor_vertices[1];
            right_light.top = armor_vertices[2];
            right_light.bottom = armor_vertices[3];
            type = armor_type;
        }
    };
}