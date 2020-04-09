/*
 * An offline implementation of weak-coupling global optimization.
 *
 *
*/
#include <iostream>
#include <fstream>
#include <string>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include "gtsam/navigation/AHRSFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam_unstable/slam/BiasedGPSFactor.h"
#include "ImuFactor.h"


#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
using namespace std;

struct states{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    gtsam::NonlinearFactorGraph inputGraph;
    gtsam::Values initialEstimate;
    gtsam::ISAM2* p_isam;

    ifstream* pFile;
    IMUHelper imu;
    double t_imu_last = -1;

    int slam_id = 0; //index of slam info.

    Eigen::Vector3d slam_pos; // last frame pose.
    Eigen::Quaterniond slam_quat;

    Eigen::Vector3d gps_buf;


    Pose3 last_optimized_pose;

    void imu_callback()
    {}
    void gps_callback()
    {}
    void isam_forward()
    {
        if(this->slam_id == 0)
        {
            return;
        }
        //initialEstimate.insert(X(this->slam_id), Pose3::identity());
        initialEstimate.insert(X(this->slam_id), last_optimized_pose);
        initialEstimate.insert(V(this->slam_id), imu.prevState.velocity()//Vector3(0, 0, 0)
                               );
        initialEstimate.insert(B(this->slam_id), imu.prevBias);

        CombinedImuFactor imuFactor(X(this->slam_id - 1), V(this->slam_id - 1),
                                    X(this->slam_id), V(this->slam_id),
                                    B(this->slam_id - 1),B(this->slam_id),
                                    *imu.preintegrated);
        inputGraph.add(imuFactor);
        auto isam_res = p_isam->update(inputGraph, initialEstimate);
        {
            if(isam_res.errorBefore&&isam_res.errorAfter)
            {
                cout<<"ISAM Error: before:"<<*isam_res.errorBefore<<";after:"<<*isam_res.errorAfter<<endl;
            }
            else
            {
                cout<<"ISAM error unknown."<<endl;
            }
        }

        Values currentEstimate = p_isam->calculateEstimate();

        imu.propState = imu.preintegrated->predict(imu.prevState, imu.prevBias);
        imu.prevState = NavState(currentEstimate.at<Pose3>(X(this->slam_id)),
                                 currentEstimate.at<Vector3>(V(this->slam_id)));
        imu.prevBias = currentEstimate.at<imuBias::ConstantBias>(B(this->slam_id));
        imu.preintegrated->resetIntegrationAndSetBias(imu.prevBias);

        inputGraph.resize(0);
        initialEstimate.clear();

        //output
        Pose3 pose = currentEstimate.at(X(slam_id)).cast<Pose3>();
        last_optimized_pose = pose;
        auto position = pose.translation();
        Vector3 velocity = currentEstimate.at(V(slam_id)).cast<Vector3>();
        cout<<"SLAM id:"<<slam_id<<";Position:"<<position.x()<<","<<position.y()<<","<<position.z()
           <<endl<<"Velocity:"<<velocity[0]<<","<<velocity[1]<<","<<velocity[2]<<endl;
    }

}
state;

std::pair<char,vector<double> > parseLine()
{
    string line;
    getline(*state.pFile,line);
    //cout<<line<<endl;
    istringstream ss(line);
    char type;
    ss>>type;

    vector<double> measurements;
    while(!ss.eof())
    {
        char comma;
        double val;
        ss>>comma;
        ss>>val;
        measurements.push_back(val);
    }
    return std::make_pair(type,measurements);
}


void initGraph()
{
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
    state.inputGraph.emplace_shared<PriorFactor<Pose3>>(X(0), Pose3::identity(),
                                               priorPoseNoise);
    state.initialEstimate.insert(X(0), Pose3::identity());

    // Bias prior
    state.inputGraph.add(PriorFactor<imuBias::ConstantBias>(B(0), state.imu.priorImuBias,
                                                   state.imu.biasNoiseModel));
    state.initialEstimate.insert(B(0), state.imu.priorImuBias);

    // Velocity prior - assume stationary
    state.inputGraph.add(
        PriorFactor<Vector3>(V(0), Vector3(0, 0, 0), state.imu.velocityNoiseModel));
    state.initialEstimate.insert(V(0), Vector3(0, 0, 0));

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.setEnableRelinearization(false);
    parameters.setEvaluateNonlinearError(true);
    state.p_isam = new gtsam::ISAM2(parameters);

}
void processLine()
{
    std::pair<char,vector<double> > data = parseLine();
    vector<double>& vVal = data.second;
    if(data.first == 'I')//imu info.
    {
        if(state.t_imu_last<0||state.slam_id ==0)//not initialized;
        {
            state.t_imu_last = vVal[0];//time.
            return;
        }
        else
        {
            double gx=vVal[1], gy=vVal[2], gz=vVal[3];
            double ax=vVal[4], ay=vVal[5], az=vVal[6];
            double dt = (vVal[0]-state.t_imu_last) / 1000000000.0;
            state.t_imu_last = vVal[0];
            Vector3 acc(ax, ay, az);
            Vector3 gyr(gx, gy, gz);
            state.imu.preintegrated->integrateMeasurement(acc, gyr, dt);
        }
    }
    else if(data.first == 'S')
    {
        double px=vVal[1],py=vVal[2],pz=vVal[3];
        double x=vVal[4],y=vVal[5],z=vVal[6],w=vVal[7];
        if(state.slam_id == 0)
        {
            state.slam_pos= Eigen::Vector3d(px,py,pz);
            state.slam_quat=Eigen::Quaterniond(w,x,y,z);
            state.slam_id++;
            return;
        }
        else
        {
            Eigen::Vector3d current_pos(px,py,pz);
            Eigen::Quaterniond current_quat(w,x,y,z);

            Eigen::Vector3d diff_pos = current_pos - state.slam_pos;
            Eigen::Quaterniond diff_quat = current_quat*state.slam_quat.inverse();
            auto gaussian = noiseModel::Diagonal::Sigmas(
                                            (Vector(6) << Vector3::Constant(5.0e-2), Vector3::Constant(5.0e-3)).finished());
            state.inputGraph.add(BetweenFactor<Pose3>(X(state.slam_id-1),X(state.slam_id),Pose3(Rot3(diff_quat),Point3(diff_pos)),gaussian));
            state.slam_pos = current_pos;
            state.slam_quat=Eigen::Quaterniond(w,x,y,z);
        }
        state.imu_callback();
        state.gps_callback();
        state.isam_forward();
        state.slam_id++;
    }
    else if(data.first == 'G')
    {
//TODO.

    }
}




int main()
{
    state.pFile = new ifstream("output.csv");
    initGraph();
    while(!state.pFile->eof())
    {
        processLine();
    }
    return 0;
}
