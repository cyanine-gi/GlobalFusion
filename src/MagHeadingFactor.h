#ifndef MAG_FACTOR_GLOBALFUSION_H
#define MAG_FACTOR_GLOBALFUSION_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>



//class MagFactor1: public NoiseModelFactor1<Rot3> {

//  const Point3 measured_; ///< The measured magnetometer values
//  const Point3 nM_; ///< Local magnetic field (mag output units)
//  const Point3 bias_; ///< bias

//public:

//  /** Constructor */
//  MagFactor1(Key key, const Point3& measured, double scale,
//      const Unit3& direction, const Point3& bias,
//      const SharedNoiseModel& model) :
//      NoiseModelFactor1<Rot3>(model, key), //
//      measured_(measured), nM_(scale * direction), bias_(bias) {
//  }

//  /// @return a deep copy of this factor
//  virtual NonlinearFactor::shared_ptr clone() const {
//    return boost::static_pointer_cast<NonlinearFactor>(
//        NonlinearFactor::shared_ptr(new MagFactor1(*this)));
//  }

//  /**
//   * @brief vector of errors
//   */
//  Vector evaluateError(const Rot3& nRb,
//      boost::optional<Matrix&> H = boost::none) const {
//    // measured bM = nRb� * nM + b
//    Point3 hx = nRb.unrotate(nM_, H, boost::none) + bias_;
//    return (hx - measured_);
//  }
//};

namespace gtsam
{


class MagHeadingFactor:public NoiseModelFactor1<Pose3>
{
private:
    const Point3 measured_; ///< The measured magnetometer values
    const Point3 nM_; ///< Local magnetic field (mag output units)
    const Point3 bias_; ///< bias
public:

    /** Constructor */
    MagHeadingFactor(Key key, const Point3& measured, double scale,
                     const Unit3& direction, const Point3& bias,
                     const SharedNoiseModel& model) :
        NoiseModelFactor1<Pose3>(model, key), //
        measured_(measured), nM_(scale * direction), bias_(bias) {
    }

    /// @return a deep copy of this factor
    virtual NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<NonlinearFactor>(
                    NonlinearFactor::shared_ptr(new MagHeadingFactor(*this)));
    }

    /**
   * @brief vector of errors
   */
    Vector evaluateError(const Pose3& p1,//nRb,
                         boost::optional<Matrix&> H = boost::none) const {
        // measured bM = nRb� * nM + b
        Matrix m;Matrix& refM=m;
        boost::optional<Matrix&> H_rotate = refM;
        Point3 hx = p1.rotation().unrotate(nM_, H_rotate, boost::none) + bias_;
        if(H)
        {
            H->resize(3,6);
            (*H).block<3,3>(0,0) = *H_rotate;
            (*H).block<3,3>(0,3) = Matrix3::Zero();
        }
        return (hx - measured_);
    }

};


}


#endif
