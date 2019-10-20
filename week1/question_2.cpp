#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>

using namespace std;

int main(int argc, char **argv)
{

     Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 1, 1).normalized()).toRotationMatrix();

     Eigen::Quaterniond q(R);


     Eigen::Vector3d omega(0.01, 0.02, 0.03);


     Eigen::Matrix3d updated_R = R * Sophus::SO3::exp(omega).matrix();
     Eigen::Quaterniond q_R(updated_R);

     cout << "通过旋转矩阵更新后的q:" << endl
          << q_R.coeffs() << endl;

     Eigen::Quaterniond q_delta(1, 0.005, 0.01, 0.015);
     q_delta.normalize();
     Eigen::Quaterniond q_q = q * q_delta;

     q_q.normalize();
     cout << "通过四元数更新后的q:" << endl
          << q_q.coeffs() << endl;
     Eigen::Vector4d diff = q_R.coeffs() - q_q.coeffs();
     cout << "以四元数形式来比较二者差距很小:" << endl
          << diff << endl;

     cout << "R1: " << endl
          << updated_R << endl;
     cout << "R2: " << endl
          << q_q.matrix() << endl;

     cout << "以旋转矩阵来比较二者差距还是很小: " << endl
          << updated_R - q_q.matrix() << endl;

     cout << "***********" << endl;
     Sophus::SO3 so3(R);
     Eigen::Quaterniond q_update(1, 0.005, 0.01, 0.015);
     q_update.normalize();

     Eigen::Vector3d update_so3(0.01, 0.02, 0.03);
     Sophus::SO3 so3_updated = so3 * Sophus::SO3::exp(update_so3);
     cout << "SO3"
          << " " << so3_updated << endl;

     Eigen::Quaterniond q_updated = q * q_update;

     Sophus::SO3 so3_q(q_updated);

     cout << "q_updated " << so3_q << endl;

     cout << Sophus::SO3(updated_R) << endl;

     cout << Sophus::SO3(q_q) << endl;


     cout << q_delta.coeffs() << endl;
     cout << q_update.coeffs() << endl;

     return 0;
}
