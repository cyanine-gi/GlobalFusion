/*
 * An offline implementation of weak-coupling global optimization.
 *
 *
*/
#include <iostream>
#include <fstream>
#include <string>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>


#include "gtsam/navigation/AHRSFactor.h"
//#include "gtsam/navigation/GPSFactor.h"
//#include "gtsam_unstable/slam/BiasedGPSFactor.h"
#include "gpsfactorwithheading.h"
#include "GPSExpand.h"
#include "ImuFactor.h"


#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include "MagHeadingFactor.h"
#include "Timer.h"
using namespace std;

struct states{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //optimizable graph.
    gtsam::NonlinearFactorGraph inputGraph;
    gtsam::Values initialEstimate;
    gtsam::ISAM2* p_isam;
    Pose3 last_optimized_pose;

    IncrementalFixedLagSmoother smootherISAM2;
    FixedLagSmoother::KeyTimestampMap newTimestamps;

    //file reading;
    ifstream* pFile;

    //imu states.
    IMUHelper imu;
    double t_imu_last = -1;
    //slam_info
    int slam_id = 0; //index of slam info.

    Eigen::Vector3d slam_pos; // last frame pose.
    Eigen::Quaterniond slam_quat;

    //gps_info
    Eigen::Vector3d gps_buf;
    bool gps_buf_valid = false;
    Eigen::Vector3d first_gps;
    GPSExpand gps_center;

    //magnetic info
    Eigen::Vector3d magnet_buf;
    bool magnet_avail = false;

    void imu_callback()
    {}
    void gps_callback()
    {
        if(gps_buf_valid)
        {
            auto gaussian = noiseModel::Diagonal::Sigmas(
                        (Vector(6) << Vector3::Constant(0.5), Vector3::Constant(0.5)).finished());
            //this->inputGraph.add(GPSFactorWithHeading(X(0),X(slam_id),Symbol('R',0),first_gps,gps_buf,gaussian));
            this->inputGraph.push_back(GPSFactorWithHeading(X(0),X(slam_id),Symbol('R',0),first_gps,gps_buf,gaussian));

            auto gaussian_initial = noiseModel::Diagonal::Sigmas(
                                            (Vector(6) << Vector3::Constant(0.8), Vector3::Constant(0.5)).finished());
            //this->inputGraph.add(PriorFactor<Pose3>(X(0),Pose3(),gaussian_initial));

            gps_buf_valid = false;
        }
    }
    void magnet_callback()
    {
        //Point3 nM(22653.29982, -1956.83010, 44202.47862);
        Point3 nM(0.703118927777,21.2741419,41.9381707907);
        // Let's assume scale factor,
        double scale = 1;//255.0 / 50000.0;
        Point3 bias(0,0,0);//(10, -10, 50);
        // ... then we measure
        Point3 scaled = scale * nM;
        //Point3 measured = (scale * nM) + bias;

        double s(scale * nM.norm());
        Unit3 direction(nM);


        magnet_buf*=1000000.0;
        cout<<"Magnet scaled"<<scaled<<";direction:"<<direction.point3()<<";measured:"<<Point3(magnet_buf)<<endl;
         //MagFactor1 f1(1, measured, s, dir, bias, model);//like gtsam example.


        if(magnet_avail)//usage:
        {
            SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 2.0);
            //this->inputGraph.add(MagHeadingFactor(X(slam_id),magnet_buf,s,direction,bias,model));
            this->inputGraph.push_back(MagHeadingFactor(X(slam_id),magnet_buf,s,direction,bias,model));
            magnet_avail = false;
        }
    }
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
        this->newTimestamps[X(this->slam_id)] = slam_id;
        this->newTimestamps[V(this->slam_id)] = slam_id;
        this->newTimestamps[B(this->slam_id)] = slam_id;
        //inputGraph.add(imuFactor);
        inputGraph.push_back(imuFactor);
        ScopeTimer t_optimization("Optimizer iterate");
        //auto isam_res = p_isam->update(inputGraph, initialEstimate);
        this->smootherISAM2.update(inputGraph,initialEstimate,newTimestamps);

        auto isam_res = this->smootherISAM2.getISAM2Result();
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


        //Values currentEstimate = p_isam->calculateEstimate();


        imu.propState = imu.preintegrated->predict(imu.prevState, imu.prevBias);
        imu.prevState = NavState(smootherISAM2.calculateEstimate<Pose3>(X(this->slam_id)),
                                 smootherISAM2.calculateEstimate<Vector3>(V(this->slam_id)));
        imu.prevBias = smootherISAM2.calculateEstimate<imuBias::ConstantBias>(B(this->slam_id));
        imu.preintegrated->resetIntegrationAndSetBias(imu.prevBias);






        //output
        Pose3 pose = smootherISAM2.calculateEstimate<Pose3>(X(slam_id));
        last_optimized_pose = pose;
        auto position = pose.translation();
        Vector3 velocity = smootherISAM2.calculateEstimate<Vector3>(V(slam_id));
        cout<<"SLAM id:"<<slam_id<<";Position:"<<position.x()<<","<<position.y()<<","<<position.z()
           <<endl<<"Velocity:"<<velocity[0]<<","<<velocity[1]<<","<<velocity[2]<<endl;
        //Visualize relative rotation;
//        if(currentEstimate.exists(Symbol('R',0)))
//        {
//            Rot2 relative_rotation = currentEstimate.at(Symbol('R',0)).cast<Rot2>();
//            cout<<"Rotation estimated:"<<relative_rotation.theta()<<endl;
//        }
        //Visualize Body Rotation
        Rot3 rot_ = pose.rotation();
        cout<<"Optimized ypr:"<<Point3(rot_.ypr()*180/3.14159)<<endl;
        Rot3 original_rot(slam_quat);
        cout<<"Original ypr:"<<Point3(original_rot.ypr()*180/3.14159)<<endl;
        cout<<"-----------"<<endl;



        t_optimization.watch("ISAM time cost:");
        inputGraph.resize(0);
        initialEstimate.clear();
        newTimestamps.clear();
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
          (Vector(6) << Vector3::Constant(100),//Rotation
            Vector3::Constant(0.1)).finished());
    state.inputGraph.emplace_shared<PriorFactor<Pose3>>(X(0), Pose3::identity(),
                                               priorPoseNoise);
    state.initialEstimate.insert(X(0), Pose3::identity());
    state.newTimestamps[X(0)] = 0;

    // Bias prior
    state.inputGraph.add(PriorFactor<imuBias::ConstantBias>(B(0), state.imu.priorImuBias,
                                                   state.imu.biasNoiseModel));
    state.initialEstimate.insert(B(0), state.imu.priorImuBias);
    state.newTimestamps[B(0)] = 0;

    // Velocity prior - assume stationary
    state.inputGraph.add(
        PriorFactor<Vector3>(V(0), Vector3(0, 0, 0), state.imu.velocityNoiseModel));
    state.initialEstimate.insert(V(0), Vector3(0, 0, 0));
    state.newTimestamps[V(0)] = 0;

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.setEnableRelinearization(true);
    parameters.setEvaluateNonlinearError(true);
    //parameters.setRelinearizeSkip(3);
    state.p_isam = new gtsam::ISAM2(parameters);

    double lag = 30.0;
    state.smootherISAM2 = IncrementalFixedLagSmoother(lag,parameters);


    //add gps_slam yaw diff symbol.
    if(!state.initialEstimate.exists(Symbol('R',0)))
    {
        state.initialEstimate.insert<Rot2>(Symbol('R',0),Rot2(0));
        auto gaussian = noiseModel::Diagonal::Sigmas(
                                        (Vector(1) <<100.0).finished());
        state.inputGraph.add(PriorFactor<Rot2>(Symbol('R',0),Rot2(0),gaussian));
    }

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
            //cout<<"dt:"<<dt<<"s."<<endl;
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
            Pose3 current_pos_abs = Pose3(Rot3(current_quat),Point3(current_pos));
            Pose3 prev_pos_abs(Rot3(state.slam_quat),Point3(state.slam_pos));


            Eigen::Matrix4d m = (prev_pos_abs.matrix().inverse());//*
            Eigen::Matrix4d n = (current_pos_abs.matrix());
            Pose3 diff_pose = prev_pos_abs.transformPoseTo(current_pos_abs);
            //Pose3 diff_pose(n*m);
            auto gaussian = noiseModel::Diagonal::Sigmas(
                                        (Vector(6) << Vector3::Constant(5.0e-2), Vector3::Constant(5.0e-2)).finished());
            //Relative.
//            {

//                state.inputGraph.add(BetweenFactor<Pose3>(X(state.slam_id-1),X(state.slam_id),diff_pose,gaussian));
//                state.slam_pos = current_pos;
//                state.slam_quat=Eigen::Quaterniond(w,x,y,z);
//            }

            //Absolute.
            {
                auto gaussian_abs = noiseModel::Diagonal::Sigmas(
                            (Vector(6) << Vector3::Constant(0.05), Vector3::Constant(0.05)).finished());
                state.inputGraph.add(PriorFactor<Pose3>(X(state.slam_id),current_pos_abs,gaussian_abs)); //DEBUG only.
            }

            auto s_pos = current_pos;
            cout<<"SLAM only:"<<s_pos[0]<<","<<s_pos[1]<<","<<s_pos[2]<<endl;

            state.slam_pos= Eigen::Vector3d(px,py,pz);
            state.slam_quat=Eigen::Quaterniond(w,x,y,z);
        }
        state.imu_callback();
        //state.gps_callback();
        state.magnet_callback();
        state.isam_forward();
        state.slam_id++;
    }
    else if(data.first == 'M')//Magnetic sensor
    {
        double mx=vVal[1],my=vVal[2],mz=vVal[3];
        state.magnet_buf = Eigen::Vector3d(mx,my,mz);
        state.magnet_avail = true;
    }
    else if(data.first == 'G')
    {
        double lon_=vVal[1],lat_=vVal[2],alt_=vVal[3];

        if(state.slam_id == 0)
        {
            state.gps_center.expandAt(lon_,lat_,alt_);
            state.first_gps = Eigen::Vector3d(0,0,0);
            return;
        }
        else
        {
            Eigen::Vector3d current_pos_gps = state.gps_center.query_input_relative_vec(lon_,lat_,alt_);
            state.gps_buf = current_pos_gps;
            cout<<"gps relative:"<<current_pos_gps<<endl;
            if(current_pos_gps.squaredNorm()>1.0)
            {
                state.gps_buf_valid = true;
            }
        }

    }
}




int main(int argc,char** argv)
{
    google::InitGoogleLogging(argv[0]);
    state.pFile = new ifstream("output.csv");
    initGraph();
    while(!state.pFile->eof())
    {
        processLine();
    }
    return 0;
}
