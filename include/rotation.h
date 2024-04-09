#ifndef _ROTATION_H_
#define _ROTATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>


class Rotation
{
private:
    /* data */
    
public:

    static Eigen::Quaterniond rotmat2quaternion(const Eigen::Matrix3d &matrix) 
    {
        return Eigen::Quaterniond(matrix);
    }

    static Eigen::Matrix3d quaternion2rotmat(const Eigen::Quaterniond &quaternion) 
    {
        return quaternion.toRotationMatrix();
    }
    
    static Eigen::Vector3d rotmat2euler(const Eigen::Matrix3d &rotmat)
    {
        Eigen::Vector3d euler = Eigen::Vector3d::Zero();
        euler[1] = atan(-rotmat(2, 0) / sqrt(rotmat(2, 1) * rotmat(2, 1) + rotmat(2, 2) * rotmat(2, 2)));
        if (rotmat(2, 0) <= -0.999) {
            euler[0] = 0;
            euler[2] = atan2((rotmat(1, 2) - rotmat(0, 1)), (rotmat(0, 2) + rotmat(1, 1)));
            std::cout << "[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!" << std::endl;
        } else if (rotmat(2, 0) >= 0.999) {
            euler[0] = 0;
            euler[2] = M_PI + atan2((rotmat(1, 2) + rotmat(0, 1)), (rotmat(0, 2) - rotmat(1, 1)));
            std::cout << "[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!" << std::endl;
        } else {
            euler[0] = atan2(rotmat(2, 1), rotmat(2, 2));
            euler[2] = atan2(rotmat(1, 0), rotmat(0, 0));
        }
        // heading 0~2PI
        if (euler[2] < 0) {
            euler[2] = M_PI * 2 + euler[2];
        }
        return euler;
    }
   
    static Eigen::Matrix3d euler2rotmat(const Eigen::Vector3d &euler)
    {
        return Eigen::Matrix3d(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
    }

    static Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond &quaternion)
    {
        return rotmat2euler(quaternion.toRotationMatrix());
    }

    static Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d &euler)
    {
        return Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
    }

    static Eigen::Quaterniond rotvec2quaternion(const Eigen::Vector3d &rotvec) 
    {
        double angle = 0.0;
        Eigen::Vector3d vec = Eigen::Vector3d::Zero();
        vec = rotvec.normalized();
        angle = rotvec.norm();
        return Eigen::Quaterniond(Eigen::AngleAxisd(angle, vec));
    }

    static Eigen::Vector3d quaternion2rotvec(const Eigen::Quaterniond &quaternion)
    {
        Eigen::AngleAxisd angle_axis = Eigen::AngleAxisd::Identity();
        Eigen::Vector3d rotvec = Eigen::Vector3d::Zero();
        angle_axis = Eigen::AngleAxisd(quaternion);
        rotvec = angle_axis.angle() * angle_axis.axis();
        return rotvec;
    }
    
    static Eigen::Matrix3d vec2skewsmt(const Eigen::Vector3d vec3)
    {
        Eigen::Matrix3d mat3 = Eigen::Matrix3d::Zero();
        mat3 <<     0,      -vec3(2),   vec3(1),
                vec3(2),        0,      -vec3(0),
                -vec3(1),   vec3(0),    0;
        return mat3;
    }

};

#endif
