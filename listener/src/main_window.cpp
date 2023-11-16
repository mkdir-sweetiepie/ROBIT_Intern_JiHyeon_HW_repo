/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/listener/main_window.hpp"
#include "QByteArray"
#include <opencv2/opencv.hpp>
#include <QImage>
#include <vector>

/*****************************************************************************
** Namespaces
*****************************************************************************/
int value1;
int value2;
int value3;
int value4;
int value5;
int value6;
int value7;
int value8;

int area1;
int left1;
int top1;
int width1;
int height1;

int area2;
int left2;
int top2;
int width2;
int height2;

namespace listener
{
  using namespace Qt;

  /*****************************************************************************
  ** Implementation [MainWindow]
  *****************************************************************************/

  MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
  {
    ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    setWindowIcon(QIcon(":/images/icon.png"));
    qnode.init();

    IP = QHostAddress("192.168.0.73");  //IP 주소와 포트 번호 설정 
    PORT = 8888;
    ocam1_UdpSocket = new QUdpSocket;  //QUdpSocket 객체 생성

    // 소켓을 지정된 IP와 포트로 바인딩하고, 바인딩이 성공하면 readyRead() 시그널이 발생하도록 함
    if (ocam1_UdpSocket->bind(IP, PORT)) 
    {
      // readyRead() 시그널이 발생하면 ocam1_read() 슬롯을 호출하도록 연결
      connect(ocam1_UdpSocket, SIGNAL(readyRead()), this, SLOT(ocam1_read()));
    }

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  }

  MainWindow::~MainWindow()
  {
  }

  /*****************************************************************************
  ** Functions
  *****************************************************************************/
  
  //수신 성공시 실행
  void MainWindow::ocam1_read()
  {
    //QByteArray 객체를 생성하여 수신된 데이터를 저장할 버퍼로 사용
    QByteArray cam_buffer;
    //수신 대기 중인 데이터의 크기에 맞게 버퍼의 크기를 조정
    cam_buffer.resize(ocam1_UdpSocket->pendingDatagramSize());
    //소켓으로부터 데이터를 읽어와서 버퍼에 저장하고, 송신자의 IP 주소와 포트 번호를 얻어옴
    ocam1_UdpSocket->readDatagram(cam_buffer.data(), cam_buffer.size(), &IP, &PORT);
    //수신된 데이터를 해독하는 함수 호출
    Decoding_Datagram_ocam1(cam_buffer);
    //버퍼를 비워줌
    cam_buffer.clear();
  }

  /*네트워크 통신에서는 데이터를 전송하기 전에 주로 인코딩하고, 
  수신 측에서는 그 데이터를 디코딩하여 원래의 형태로 복원합니다. 
  이는 데이터를 효율적으로 전송하거나, 특정 형식으로 표현하는 데 사용됩니다.

  이미지 데이터를 udp 송신을 위해 변형시켜야 하는데 
  처음에 언급했던 것처럼 raw_image를 그대로 전송하기에는 전송량이 매우 크기 때문에 
  이미지 압축을 opencv에서 제공하는 imencode 함수를 통해 진행합니다.

  압축한 카메라 이미지는 vector<uchar> 형식이기 때문에 
  udp 전송을 위해 QByteArray로의 형변환을 진행해야 합니다. 
  이후 writeDatagram 함수로 이미지를 목표 IP, port를 향해 최종적으로 전송할 수 있을 것입니다.
  수신은 그 반대입니다. */
  void MainWindow::Decoding_Datagram_ocam1(QByteArray inputDatagram)
  {
    /////////////////////decode & resize///////////////////////
    //QUdpSocket을 통해 전송된 이미지 데이터를 QByteArray로 변환
    std::vector<uchar> ocam1_decoding(inputDatagram.begin(), inputDatagram.end());
    //QByteArray를 사용하여 QImage를 생성하고, QLabel에 원본 이미지 표시
    QImage qImage = QImage::fromData(inputDatagram);
    ui.label->setPixmap(QPixmap::fromImage(qImage));

    ocam1_image = cv::imdecode(cv::Mat(ocam1_decoding), cv::IMREAD_COLOR);  //이미지 디코딩
    cv::Size newSize(360, 260);  //새로운 크기 설정
    cv::resize(ocam1_image, ocam1_image, newSize);  //이미지 재설정
    //OpenCV 이미지를 QImage로 변환하고, 색상 순서를 조정하여 리사이즈된 이미지를 QLabel에 표시
    QImage resizedQImage(ocam1_image.data, ocam1_image.cols, ocam1_image.rows, ocam1_image.step, QImage::Format_RGB888);
    ui.label->setPixmap(QPixmap::fromImage(resizedQImage.rgbSwapped()));

    //////////////////이진화 찾기//////////////////////
    bi();

    //하얀색 이진화 찾기 (선)
    cv::Mat grayImage;
    cv::cvtColor(ocam1_image, grayImage, cv::COLOR_BGR2HSV);
    // Apply Gaussian blur to reduce reflection
    cv::Mat blurredImage;
    cv::GaussianBlur(grayImage, blurredImage, cv::Size(5, 5), 0);
    
    // 이미 HSV 이미지로 변환한 변수를 사용
    cv::Scalar lower_white(0, 0, 193);
    cv::Scalar upper_white(180, 55, 255);
    // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
    cv::Mat white;
    cv::inRange(blurredImage, lower_white, upper_white, white);
    // 이진 이미지를 표시
    cv::Mat white_img = region_of_interest(white); 
    QImage whiteImage(white_img.data, white_img.cols, white_img.rows, white_img.step, QImage::Format_Grayscale8);
    ui.label_11->setPixmap(QPixmap::fromImage(whiteImage));
    
    // 이미 HSV 이미지로 변환한 변수를 사용
    cv::Scalar lower_orange(0, 42, 74);
    cv::Scalar upper_orange(25, 255, 255);
    // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
    cv::Mat orange;
    cv::inRange(blurredImage, lower_orange, upper_orange, orange);
    // 이진 이미지를 표시
    QImage orangeImage(orange.data, orange.cols, orange.rows, orange.step, QImage::Format_Grayscale8);
    ui.label_14->setPixmap(QPixmap::fromImage(orangeImage));

    // 이미 HSV 이미지로 변환한 변수를 사용
    cv::Scalar lower_green(37, 83, 143);
    cv::Scalar upper_green(43, 156, 255);
    // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
    cv::Mat green;
    cv::inRange(blurredImage, lower_green, upper_green, green);
    // 이진 이미지를 표시
    QImage greenImage(green.data, green.cols, green.rows, green.step, QImage::Format_Grayscale8);
    ui.label_15->setPixmap(QPixmap::fromImage(greenImage));


    cv::Mat edge = canny_edge(white);
    QImage edge_image(edge.data, edge.cols, edge.rows, edge.step, QImage::Format_Grayscale8);
    ui.label_2->setPixmap(QPixmap::fromImage(edge_image));
    cv::Mat roi_conversion = region_of_interest(edge); 
    QImage roi_conversion_image(roi_conversion.data, roi_conversion.cols, roi_conversion.rows, roi_conversion.step, QImage::Format_Grayscale8);
    ui.label_3->setPixmap(QPixmap::fromImage(roi_conversion_image));



    cv::Mat orangelabels,greenlabels, stats, centroids;
    int numorange = connectedComponentsWithStats(orange, orangelabels, stats, centroids, 4, CV_32S);
    // 오렌지 레이블링 결과에 사각형 그림
    for (int j = 1; j < numorange; j++) 
    {
      area1 = stats.at<int>(j, cv::CC_STAT_AREA);
      left1 = stats.at<int>(j, cv::CC_STAT_LEFT);
      top1 = stats.at<int>(j, cv::CC_STAT_TOP);
      width1 = stats.at<int>(j, cv::CC_STAT_WIDTH);
      height1 = stats.at<int>(j, cv::CC_STAT_HEIGHT);

      rectangle(ocam1_image, cv::Point(left1, top1),cv::Point(left1 + width1, top1 + height1),cv::Scalar(255, 0, 0), 1);
    }

    int numgreen= connectedComponentsWithStats(green, greenlabels, stats, centroids, 4, CV_32S);
    // 그린 레이블링 결과에 사각형 그림
    for (int j = 1; j < numgreen; j++) 
    {
      area2 = stats.at<int>(j, cv::CC_STAT_AREA);
      left2 = stats.at<int>(j, cv::CC_STAT_LEFT);
      top2 = stats.at<int>(j, cv::CC_STAT_TOP);
      width2 = stats.at<int>(j, cv::CC_STAT_WIDTH);
      height2 = stats.at<int>(j, cv::CC_STAT_HEIGHT);

      rectangle(ocam1_image, cv::Point(left2, top2),cv::Point(left2 + width2, top2 + height2),cv::Scalar(0, 255, 0), 1);
    }

    //관심영역에서 선 추출해서 선 그림
    cv::Rect roi(5, roi_conversion.rows / 2 - 6, 275, 21);    
    cv::Mat roiImage = roi_conversion(roi);
    //Hough Line Transform을 사용하여 선 추출
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(roiImage, lines, 1, CV_PI / 180, 50, 30, 60);    
    //선을 그릴 원본 이미지 복사
    cv::Mat outputImage = ocam1_image.clone();
    //추출된 선을 이미지에 그리기
    for (size_t i = 0; i < lines.size(); i++)
    {
      cv::Vec4i line = lines[i];
      cv::line(outputImage, cv::Point(line[0] + roi.x, line[1] + roi.y), cv::Point(line[2] + roi.x, line[3] + roi.y), cv::Scalar(0, 0, 255), 2);
    }
    //UI에 선이 그려진 이미지 표시
    QImage outputQImage(outputImage.data, outputImage.cols, outputImage.rows, outputImage.step, QImage::Format_RGB888);
    ui.label_16->setPixmap(QPixmap::fromImage(outputQImage.rgbSwapped()));

    //콘의 위치를 분석하는 함수 호출
    analyzeConesPosition();
  }

  //이미지에 Canny 엣지 검출을 적용하는 함수
  cv::Mat MainWindow::canny_edge(cv::Mat image)
  {
    //이미지를 가우시안 블러로 먼저 전처리
    cv::Mat blur_conversion;
    cv::GaussianBlur(image, blur_conversion, cv::Size(5, 5), 0);

    // 가우시안 블러된 이미지에 Canny 엣지 검출 적용
    cv::Mat canny_conversion;
    cv::Canny(blur_conversion, canny_conversion, 50, 200); //7,8

    value7 = ui.horizontalSlider_7->value();
    value8 = ui.horizontalSlider_8->value();

    ui.label_12->setText(QString::number(value7));
    ui.label_13->setText(QString::number(value8));

    return canny_conversion;
  }

  //관심 영역 설정 함수
  cv::Mat MainWindow::region_of_interest(cv::Mat image)
  {
    //입력 이미지의 높이와 너비 가져오기
    int imageHeight = image.rows;
    int imageWidth = image.cols;
    //이미지 크기와 같은 빈 이미지 생성
    cv::Mat imageMask = cv::Mat::zeros(image.size(), image.type());

    cv::Point vertices[4]; //관심 영역의 꼭지점 좌표 설정
    vertices[0] = cv::Point(5, imageHeight/2+10); //좌측 하단 꼭지점
    vertices[1] = cv::Point(280, imageHeight/2+17); //우측 하단 꼭지점
    vertices[2] = cv::Point(280, imageHeight/2+2); //우측 상단 꼭지점
    vertices[3] = cv::Point(8, imageHeight/2-6); //좌측 상단 꼭지점

    cv::Point roi_pts[1][4];  //꼭지점을 사용하여 다각형 정의
    roi_pts[0][0] = vertices[0];
    roi_pts[0][1] = vertices[1];
    roi_pts[0][2] = vertices[2];
    roi_pts[0][3] = vertices[3];

    const cv::Point* ppt[1] = {roi_pts[0]};  //꼭지점 배열을 포인터
    int npt[] = {4}; //꼭지점 수

    //다각형이 채워질 대상 이미지, 꼭지점을 지정하는 포인터 배열, 꼭지점 수, 다각형의 수, 색상
    cv::fillPoly(imageMask, ppt, npt, 1, cv::Scalar(255, 255, 255));
    cv::Mat maskingImage; //결과이미지 
    cv::bitwise_and(image, imageMask, maskingImage);  //이미지와 마스크를 AND 연산하여 관심 영역만 추출

    return maskingImage;
  }

  //이진화 찾는 함수
  void MainWindow::bi(void)
  {
    //ocam1_image를 HSV 색상 공간으로 변환하여 hsvImage에 저장
    cv::Mat hsvImage;
    cv::cvtColor(ocam1_image, hsvImage, cv::COLOR_BGR2HSV);

    //반사를 줄이기 위해 가우시안 블러 적용
    cv::Mat blurredImage;
    cv::GaussianBlur(hsvImage, blurredImage, cv::Size(5, 5), 0);

    //이미 HSV 이미지로 변환한 변수를 사용
    cv::Scalar lower(value1, value2, value3);
    cv::Scalar upper(value4, value5, value6);

    //HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
    cv::Mat binaryImage;
    cv::inRange(blurredImage, lower, upper, binaryImage);

    //이진 이미지를 표시
    QImage binaryQImage(binaryImage.data, binaryImage.cols, binaryImage.rows, binaryImage.step, QImage::Format_Grayscale8);
    ui.label_4->setPixmap(QPixmap::fromImage(binaryQImage));

    value1 = ui.horizontalSlider_1->value(); //low hue
    value2 = ui.horizontalSlider_2->value(); //low 
    value3 = ui.horizontalSlider_3->value(); //low 
    value4 = ui.horizontalSlider_4->value(); //high 
    value5 = ui.horizontalSlider_5->value(); //high 
    value6 = ui.horizontalSlider_6->value(); //high 

    ui.label_5->setText(QString::number(value1));
    ui.label_6->setText(QString::number(value2));
    ui.label_7->setText(QString::number(value3));
    ui.label_8->setText(QString::number(value4));
    ui.label_9->setText(QString::number(value5));
    ui.label_10->setText(QString::number(value6));
  }

  //선과 콘 비교해서 콘 위치 찾기
  void MainWindow::analyzeConesPosition()
  {
    //중앙의 흰선을 기준으로 콘의 위치를 판별
    cv::Point white_line_position = findWhiteLinePosition(ocam1_image);
    //선 좌표를 관심 영역에서 가져옴
    int line_y = white_line_position.y;

    //라벨링 하단부 위치 가져옴
    int orange_cone_y = top1+height1;
    //주황색 콘의 위치가 있는 경우에만 표시
    if (orange_cone_y != -1)
    {
        if (orange_cone_y < line_y)
          ui.label_19->setText("주황색 콘은 선 위쪽에 있습니다.");
        else if (orange_cone_y > line_y + 26)
          ui.label_19->setText("주황색 콘은 선 아래쪽에 있습니다.");
        else
          ui.label_19->setText("주황색 콘은 선 위에 있습니다.");
    }
    else
    {
      ui.label_19->setText("주황색 콘을 찾지 못했습니다.");
    }

    //라벨링 하단부 위치 가져옴
    int green_cone_y = top2+height2;
    //녹색 콘의 위치가 있는 경우에만 표시
    if (green_cone_y != -1)
    {
        if (green_cone_y < line_y)
          ui.label_20->setText("초록색 콘은 선 위쪽에 있습니다.");
        else if (green_cone_y > line_y + 15)
          ui.label_20->setText("초록색 콘은 선 아래쪽에 있습니다.");
        else
          ui.label_20->setText("초록색 콘은 선 위에 있습니다.");
    }
    else
    {
      ui.label_20->setText("초록색 콘을 찾지 못했습니다.");
    }
  }

  //흰선의 중앙 위치를 찾는 함수
  cv::Point MainWindow::findWhiteLinePosition(const cv::Mat& image)
  {
    // 관심 영역 설정
    cv::Rect roi(5, ocam1_image.rows / 2 - 6, 275, 21);
    cv::Mat roiImage = image(roi);

    // 관심 영역에서 중앙의 흰선을 찾아 좌표 반환
    cv::Point center(roiImage.cols / 2, roiImage.rows / 2 + ocam1_image.rows / 2 - 6);

    return center;
  }
  cv::Point MainWindow::findConePosition(const cv::Mat& image, const QString& cone_color)
{
    // 이미지에서 해당 색상의 콘을 찾아 좌표 반환
    cv::Scalar lower_color, upper_color;

    if (cone_color == "orange")
    {
        lower_color = cv::Scalar(0, 42, 74);
        upper_color = cv::Scalar(25, 255, 255);
    }
    else if (cone_color == "green")
    {
        lower_color = cv::Scalar(36, 83, 122);
        upper_color = cv::Scalar(80, 255, 255);
    }

    cv::Mat color_mask;
    cv::inRange(image, lower_color, upper_color, color_mask);

    cv::Mat points;
    cv::findNonZero(color_mask, points);

    if (!points.empty())
    {
        cv::Rect bounding_rect = cv::boundingRect(points);
        cv::Point center(bounding_rect.x + bounding_rect.width / 2, bounding_rect.y + bounding_rect.height / 2);
        return center;
    }

    return cv::Point(-1, -1);  // 콘을 찾지 못한 경우
}
  

}  // namespace s
