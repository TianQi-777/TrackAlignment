#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv)
{
    vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses1;
    vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses2;
    vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses2_corrected;
    ifstream fin("../compare.txt");

    while (fin)  //读取真实轨迹参数
    {
        double data1[16] = {0};  //其中data1[0]表示时间，跳过（略去）
        for (auto &d1:data1)
            fin >> d1;

        Eigen::Vector3d t1(data1[1], data1[2], data1[3]);    //获取平移数据

        Eigen::Quaterniond q1(data1[7], data1[4], data1[5], data1[6]);    //获取旋转数据，注意输入是实1,虚1,虚2,虚3

        Eigen::Vector3d t2(data1[9], data1[10], data1[11]);  //其中data1[8]表示时间，跳过（略去）

        Eigen::Quaterniond q2(data1[15], data1[12], data1[13], data1[14]);

        Sophus::SE3 T1(q1, t1);  //获取李群一组（行）数据的SE3

        Sophus::SE3 T2(q2, t2);  //获取李群一组（行）数据的SE3

        poses1.push_back(T1);
        poses2.push_back(T2);

    }
    poses1.pop_back();  //删除最后一个000的元素
    poses2.pop_back();
    Vector3d sum_temp1_Vector3d;  //最终保存质心坐标（平均），中间有临时保存求和总量
    Vector3d sum_temp2_Vector3d;
    vector<Vector3d> poses1_translation;  //存储每个点的平移分量迭代器
    vector<Vector3d> poses2_translation;
    for(auto data1:poses1)
    {
        Sophus::SE3 temp1=data1;
        Vector3d temp1_Vector3d=temp1.translation();   //取pose1中的平移分量
        poses1_translation.push_back(temp1_Vector3d);  //存入平移分量迭代器
        sum_temp1_Vector3d+=temp1_Vector3d;            //对质心坐标求和，还没有取平均  注意：Vector3d可以整体加减除常数操作
    }
    for(auto data2:poses2)
    {
        Sophus::SE3 temp2=data2;
        Vector3d temp2_Vector3d=temp2.translation();
        poses2_translation.push_back(temp2_Vector3d);
        sum_temp2_Vector3d+=temp2_Vector3d;
    }
    int N = poses1.size();
    sum_temp1_Vector3d = sum_temp1_Vector3d/N;  //质心坐标
    sum_temp2_Vector3d = sum_temp2_Vector3d/N;  //可能有红线，但仍可正确计算

    vector<Vector3d> pose1_i(N),pose2_i(N); //指定迭代器初始大小,因为想用[]直接操作迭代器
    for(int i=0;i<N;i++)
    {
        pose1_i[i]= poses1_translation[i] - sum_temp1_Vector3d ;  //求去质心坐标
        pose2_i[i]= poses2_translation[i] - sum_temp2_Vector3d ;
    }
    // compute q1*q2^T
    Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W +=  pose1_i[i]*(pose2_i[i].transpose());
    }
    cout<<"W="<<W<<endl;

    // SVD on W
    JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    Matrix3d R = U* ( V.transpose() );
    Vector3d t = sum_temp1_Vector3d - R * sum_temp2_Vector3d;

    Sophus::SE3 Transform(R,t);
    Sophus::SE3 poses2_corrected_single;
    for(auto data:poses2)
    {
        poses2_corrected_single = Transform.operator*(data);  //SE3层面上的直接乘法定义
        poses2_corrected.push_back(poses2_corrected_single);  //校正后保存
    }

    DrawTrajectory(poses1,poses2_corrected); //校正后的两组图像
    //DrawTrajectory(poses1,poses2);  //读入两组数据对比轨迹差异
    return 0;
}

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1
                   ,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
    if (poses1.empty() && poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }
    
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {   //因为没有构成回环，这里减2更好，不然会连成一条直线，若为回环就-1，也可以用pop_back()
            glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses2.size() - 1; i++) {
            glColor3f(1 - (float) i / poses2.size(), 0.0f, (float) i / poses2.size());
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}