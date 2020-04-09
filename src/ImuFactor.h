#ifndef IMUFACTOR_GLOBALFUSION_H
#define IMUFACTOR_GLOBALFUSION_H
#include "gtsam/navigation/PreintegrationParams.h"
#include "gtsam/navigation/CombinedImuFactor.h"
using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;


struct IMUHelper {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUHelper() {
    {
      auto gaussian = noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(5.0e-2), Vector3::Constant(5.0e-3))
              .finished());
      auto huber = noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(1.345), gaussian);

      biasNoiseModel = huber;
    }

    {
      auto gaussian = noiseModel::Isotropic::Sigma(3, 0.01);
      auto huber = noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(1.345), gaussian);

      velocityNoiseModel = huber;
    }

    // expect IMU to be rotated in image space co-ords
    //Vector3 g__(0.0, 9.8, 0.0);
    Vector3 g__(0.0, 0.0, 9.8);
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p(new PreintegratedCombinedMeasurements::Params( g__));

    p->accelerometerCovariance =
        I_3x3 * pow(0.0565, 2.0);  // acc white noise in continuous
    p->integrationCovariance =
        I_3x3 * 1e-9;  // integration uncertainty continuous
    p->gyroscopeCovariance =
        I_3x3 * pow(4.0e-5, 2.0);  // gyro white noise in continuous
    p->biasAccCovariance = I_3x3 * pow(0.00002, 2.0);  // acc bias in continuous
    p->biasOmegaCovariance =
        I_3x3 * pow(0.001, 2.0);  // gyro bias in continuous
    p->biasAccOmegaInt = Matrix::Identity(6, 6) * 1e-5;

    // body to IMU rotation
//    Rot3 iRb(0.036129, -0.998727, 0.035207,
//             0.045417, -0.033553, -0.998404,
//             0.998315, 0.037670, 0.044147);
    //Rot3 iRb(1,0,0,0,1,0,0,0,1.0);
//data:  [0, 0, 1, 0,
//28            -1, 0, 0, 0.12,
//29            0, -1, 0, -0.3,
//30            0, 0, 0, 1.]

    //Rot3 iRb(1,0,0,0,1,0,0,0,1);
    //Rot3 iRb(1,0,0,0,0,-1,0,1,0);
//    Rot3 iRb(0,1,0,-1,0,0,0,0,1);
    Rot3 iRb(0,-1,0,1,0,0,0,0,1);

    // body to IMU translation (meters)
    Point3 iTb(0.03, -0.025, -0.06);

    // body in this example is the left camera
    p->body_P_sensor = Pose3(iRb, iTb);

    Rot3 prior_rotation = Rot3(I_3x3);
    Pose3 prior_pose(prior_rotation, Point3(0, 0, 0));

    Vector3 acc_bias(0.0, -0.0942015, 0.0);  // in camera frame
    Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);

    priorImuBias = imuBias::ConstantBias(acc_bias, gyro_bias);

    prevState = NavState(prior_pose, Vector3(0, 0, 0));
    propState = prevState;
    prevBias = priorImuBias;

    preintegrated = new PreintegratedCombinedMeasurements(p, priorImuBias);
  }

  imuBias::ConstantBias priorImuBias;  // assume zero initial bias
  noiseModel::Robust::shared_ptr velocityNoiseModel;
  noiseModel::Robust::shared_ptr biasNoiseModel;
  NavState prevState;
  NavState propState;
  imuBias::ConstantBias prevBias;
  PreintegratedCombinedMeasurements* preintegrated;
};


#endif

