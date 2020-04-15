#ifndef GPSFACTORWITHHEADING_H
#define GPSFACTORWITHHEADING_H
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "gtsam/navigation/MagFactor.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/base/Vector.h"

namespace gtsam {
using std::cout;
using std::endl;
class GTSAM_EXPORT GPSFactorWithHeading: public NoiseModelFactor3<Pose3,Pose3,Rot2>
{
    typedef NoiseModelFactor3<Pose3,Pose3,Rot2> Base;
public:
    GPSFactorWithHeading(Key poseKeyx1,Key poseKeyx2,Key rotKey,Point3 gps1,Point3 gps2,const SharedNoiseModel& model):
                 Base(model,poseKeyx1,poseKeyx2,rotKey),gps_measurement1(gps1),gps_measurement2(gps2)
    {
    }
    static Point3 unrotate(const Rot2& R, const Point3& p,
        boost::optional<Matrix&> HR = boost::none) {
      Point3 q = Rot3::Yaw(R.theta()).unrotate(p, HR, boost::none);
      if (HR) {
        // assign to temporary first to avoid error in Win-Debug mode
        Matrix H = HR->col(2);
        *HR = H;
      }
      return q;
    }
    Vector evaluateError(const Pose3& pose1, const Pose3& pose2,const Rot2& heading_diff,
                          boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none,boost::optional<Matrix&> H3 = boost::none) const {
        Eigen::Matrix3d rotate_mat3;
        rotate_mat3.block<2,2>(0,0)<<heading_diff.matrix();
        rotate_mat3(2,2) = 1;
        if (H1 || H2 || H3)
        {
            H1->resize(6,6); // jacobian wrt pose1
            //Inside LogMap(Pose3): rotation first!
            cout<<"H1.rows:"<<H1->rows()<<endl;
            (*H1).block<3,3>(3,3) = pose1.rotation().matrix();//point1 to pose1.translation.
            (*H1).block<3,3>(3,0) = Matrix3::Zero(); //point1 to pose1.rotation. not related.

            (*H1).block<3,3>(0,0) = Matrix3::Zero();//Matrix3::Identity();
            (*H1).block<3,3>(0,3) = Matrix3::Zero();

            H2->resize(6,6); // jacobian wrt pose2
            cout<<"H2.rows:"<<H2->rows()<<endl;
            (*H2).block<3,3>(3,3) = pose2.rotation().matrix()*rotate_mat3;//Matrix3::Identity();
            (*H2).block<3,3>(3,0) = Matrix3::Zero();

            (*H2).block<3,3>(0,0) = Matrix3::Zero();//Matrix3::Identity();
            (*H2).block<3,3>(0,3) = Matrix3::Zero();
            cout<<"H1:"<<(*H1)<<endl<<"H2:"<<(*H2)<<endl<<"H3:"<<(*H3)<<endl;


            {
                // measured bM = pose2 - pose1.translation.
                auto measured_  = pose2.translation() - pose1.translation();
                H3->resize(6,1);
                Matrix tmp;
                Matrix& tmp2 = tmp;
                boost::optional<Matrix&> H_heading = tmp2;
                H_heading->resize(3,1);
                Point3 hx = unrotate(heading_diff, gps_measurement2 - gps_measurement1, H_heading);
                cout<<"H3.rows:"<<H3->rows()<<"x cols:"<<H3->cols()<<endl;

                (*H3) = (Vector(6) << -1.0*(*H_heading), (*H_heading)).finished();//(part heading/a == part heading/(a-b) * part(a-b)/a)
                cout<<"H3:"<<(*H3)<<endl;
                //Point3 hx = unrotate(heading_diff, gps_measurement2 - gps_measurement1, H_heading);
                //(*H3) = *H_heading;
                //return (hx - measured_);
            }
        }
        Vector retval;
        retval.resize(6,1);
        retval.block<3,1>(0,0) = (pose1.translation() - gps_measurement1);
        retval.block<3,1>(3,0) = (pose2.translation() - rotate_mat3*gps_measurement2);
        return retval;
    }
private:
    Point3 gps_measurement1,gps_measurement2;
};



}



#endif // GPSFACTORWITHHEADING_H
