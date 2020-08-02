#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG 57.2957795130823 // 弧度转换为角度参数

#include "Kine.h"
// #include <vector>
#include <iostream>
using namespace std;

int inverse_solution()
{
    // vector<int>data;
    double Robot_Link_Len[6] = {0.1764,0.2568,0.2932,0.2932,0.2568,0.1764}; //robot link length

    Kine_CR_FiveDoF_G1 Biped5d; // robot based on the gripper0 to get inverse solution
    Biped5d.Set_Length(Robot_Link_Len);
    cout << "ok" << endl;

    return 0;
}