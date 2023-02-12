/*********************************************************************************************************************************************
* If you want to execute this file successfully, you need to install Eigen3 in your system.
* @file      Kalman_Filter.cpp                       
* @brief     
* @author    Daxia_Su
* @email     daxia-su@qq.com
* @version   1.0
* @date      2023-02-12  
Python code：https://github.com/liuchangji/2D-Kalman-Filter-Example_Dr_CAN_in_python
Dr_can's course links：https://www.bilibili.com/video/BV1ez4y1X7eR/?spm_id_from=333.999.0.0&vd_source=f9ae7beffa7e639a0e8f63cf744cf6f5
*********************************************************************************************************************************************/
#include<bits/stdc++.h>
#include<Eigen/Dense>

using namespace Eigen;
using namespace std;

#define PI 3.1415926535
// #define eye Eigen::MatrixXd::Identity(2, 2)

int time_x = 0; // 进函数的频率太快

Eigen::Matrix<float,2,1> gaussian_distribution_generator(float exp, Eigen::Matrix<float,2,2> var)
{
    int x;
    float A, B, C, r;
    float uni[2];
    Eigen::Matrix<float,2,1> result;
    srand((unsigned)time(NULL));
    for(int j = 0; j < 2; j++)
    {
        for(int i = 0; i < 2; i++, time_x += 638)
        {
            x = rand() + time_x;
            x = x%1000;
            uni[i] = (float)x;
            uni[i] /= 1000.0;
        }
        A = sqrt((-2)*log(uni[0]));
        B = 2 * PI * uni[1];
        C = A * cos(B);
        r = exp + C * var(j,j);
        result(j,0) = r;
    }
    return result;
}

class KF
{
private:
    Eigen::Matrix<float, 2, 2> A, B, H, Q, R;
public:
    void setA(float a1, float a2, float a3, float a4)
    {
        A << a1, a2,
             a3, a4;
        std::cout << "state-transition matrix A = " << A << endl;
    }
    void setB(float b1, float b2, float b3, float b4)
    {
        B << b1, b2,
             b3, b4;
        std::cout << "Input control matrix B = " << B << endl;
    }
    void setH(float h1, float h2, float h3, float h4)
    {
        H << h1, h2,
             h3, h4;
        std::cout << "state-observation matrix H = " << H << endl;
    }
    void setQ(float q)
    {
        Q << q, 0,
             0, q;
        std::cout << "covariance matrix of PN Q = " << Q << endl;
    }
    void setR(float r)
    {
        R << r, 0,
             0, r;
        std::cout << "covariance matrix of MN R = " << R << endl;
    }
    Eigen::Matrix<float,2,2> readA()
    {
        return A;
    }
    Eigen::Matrix<float,2,2> readH()
    {
        return H;
    }
    Eigen::Matrix<float,2,2> readQ()
    {
        return Q;
    }
    Eigen::Matrix<float,2,2> readR()
    {
        return R;
    }
};

int main(int argc, char** argv)
{
    //-----------------------Initialize------------------------//
    KF person;
    person.setA(1,1,0,1);
    person.setH(1,0,0,1);
    person.setQ(0.1);
    person.setR(1);
    //noise
    Eigen::Matrix<float,2,1> w,v;
    // Initialize position and velocity
    Eigen::Matrix<float,2,1> x_0;
    x_0 << 0, 1;
    // Initialize the convariance matrix of error
    Eigen::Matrix<float,2,2> p_0;
    p_0 << 1, 0, 1, 0;

    Eigen::Matrix<float,2,1> x_firstest;

    Eigen::Matrix<float,2,1> x_true = x_0;
    Eigen::Matrix<float,2,1> z_measure = x_0;
    Eigen::Matrix<float,2,1> x_postest = x_0;
    Eigen::Matrix<float,2,2> p_postest = p_0;
    Eigen::Matrix<float,2,2> p_firstest;
    // Kalman Gain
    Eigen::Matrix<float, 2, 2> K_k1, K_k2, K_k;
    // eye
    Eigen::Matrix<float, 2, 2> eye;
    eye << 1,0,0,1;

    for(int i = 0; i < 30; i++)
    {
        //-------------------generate the true value of position and velocity-------------------//
        w = gaussian_distribution_generator(0, person.readQ());
        x_true = (person.readA() * x_true) + w;
        //-------------------generate the value of the observation------------------------------//
        v = gaussian_distribution_generator(0, person.readR());
        z_measure = person.readH() * x_true + v;

        //-------------------calculate the firstest---------------------------------------------//
        x_firstest = person.readA() * x_postest;
        p_firstest = person.readA() * p_postest * person.readA().transpose() + person.readQ();
        //-------------------calculate the Kalman Gain------------------------------------------//
        K_k1 = p_firstest * person.readH().transpose();
        K_k2 = person.readH() * p_firstest * person.readH().transpose() + person.readR();
        K_k = K_k1 * K_k2.inverse();
        // //-------------------calculate the postest---------------------------------------------//
        x_postest = x_firstest + K_k * (z_measure - person.readH()*x_firstest);
        p_postest = (eye - K_k * person.readH()) * p_firstest;
    }

    system("pause");
    return 0;
}