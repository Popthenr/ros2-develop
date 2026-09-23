#include <opencv2/opencv.hpp>
#include <vector>

int main() {
    
    //读取图片
    auto img = cv::imread("微信图片_20240226143855.jpg");
    
    //检测是否读取成功
    if (img.empty())
    {
        return -1;
    }
    
    //转化成灰度
    cv::Mat gray;
    cv::cvtColor(img,gray, cv::COLOR_RGB2GRAY);

    //二值化
    cv::Mat binary;
    cv::threshold(gray,binary,127,255, cv::THRESH_BINARY);

    //查找轮廓
    std::vector<std::vector<cv::Point>>contours;
    cv::findContours(
        binary,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    //生成轮廓
    cv::Mat result = img.clone();

    cv::drawContours(
        result,
        contours,
        -1,
        cv::Scalar(0,0,255),
        2
    );
    
    //成果展示
    cv::imshow("Original",img);
    cv::imshow("Binary",binary);
    cv::imshow("Contours",result);

    cv::waitKey(0);
    return 0;
}