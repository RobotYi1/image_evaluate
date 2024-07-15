#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

double computeSharpness(const cv::Mat &image)
{
    cv::Mat gray, laplacian;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = image.clone();
    }
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
}

double computeNoise(const cv::Mat &image)
{
    cv::Mat gray, noise;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = image.clone();
    }
    cv::medianBlur(gray, noise, 3);
    cv::Mat noise_diff = gray - noise;
    cv::Scalar mean, stddev;
    cv::meanStdDev(noise_diff, mean, stddev);
    return stddev[0];
}

double computeContrast(const cv::Mat &image)
{
    cv::Mat gray;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = image.clone();
    }
    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);
    return stddev[0];
}

bool isNight(const cv::Mat &image, double brightness_threshold = 50.0, double noise_threshold = 10.0, double contrast_threshold = 30.0)
{
    cv::Mat gray;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = image.clone();
    }

    // Compute brightness
    cv::Scalar mean_brightness = cv::mean(gray);

    // Compute noise
    double noise = computeNoise(image);

    // Compute contrast
    double contrast = computeContrast(image);

    // Check if it is night based on thresholds
    return mean_brightness[0] < brightness_threshold && noise < noise_threshold && contrast < contrast_threshold;
}

// void imageCallback(const sensor_msgs::ImageConstPtr &msg)
// {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     cv::Mat image = cv_ptr->image;
//     double sharpness = computeSharpness(image);
//     double noise = computeNoise(image);
//     double contrast = computeContrast(image);

//     cv::Mat result(100, 300, CV_8UC3, cv::Scalar(255, 255, 255));
//     std::string text = "Sharpness: " + std::to_string(sharpness);
//     cv::putText(result, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
//     text = "Noise: " + std::to_string(noise);
//     cv::putText(result, text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
//     text = "Contrast: " + std::to_string(contrast);
//     cv::putText(result, text, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

//     cv::imshow("Image Quality", result);
//     cv::waitKey(1);
// }

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;
    bool night = isNight(image);
    cv::Scalar mean_brightness = cv::mean(image);
    double noise = computeNoise(image);
    double contrast = computeContrast(image);
    // Show the current image with night detection result
    cv::Mat result = image.clone();
    std::string text = night ? "Night" : "Day";
    cv::putText(result, "Time of Day: " + text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    text = "Noise: " + std::to_string(noise);
    cv::putText(result, text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    text = "Contrast: " + std::to_string(contrast);
    cv::putText(result, text, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    text = "brightness: " + std::to_string(mean_brightness[0]);
    cv::putText(result, text, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    cv::imshow("Night Detection", result);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_quality_evaluator");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/left_image", 1, imageCallback);
    ros::spin();
    return 0;
}
