#ifndef GPS_EXPAND_H
#define GPS_EXPAND_H
#include "Eigen/Core"
class GPSExpand
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void expandAt(double lon_deg,double lat_deg,double alt)
    {
      this->center_lon = lon_deg;
      this->center_lat = lat_deg;
      this->center_alt = alt;
    }
    inline double vari_km_per_lat_deg()                                           
    {
        return earth_radius_km*pi_/180;
    }
    inline double vari_km_per_lon_deg()
    {
        return vari_km_per_lat_deg()*cos(center_lat*pi_/180);
    }
    inline double getLon()
    {
        return this->center_lon;
    }
    inline double getLat()
    {
        return this->center_lat;
    }
    inline double getAlt()
    {
        return this->center_alt;
    }
    inline Eigen::Vector3d query_input_relative_vec(double lon,double lat,double alt)
    {
        return Eigen::Vector3d((lon-this->center_lon)*vari_km_per_lon_deg()*1000,(lat-this->center_lat)*vari_km_per_lat_deg()*1000,alt-this->center_alt);
    }
private:
    double center_lon;
    double center_lat;
    double center_alt;
    const double earth_radius_km = 6371.393;
    const double pi_ = 3.1415926535;

};
#endif
