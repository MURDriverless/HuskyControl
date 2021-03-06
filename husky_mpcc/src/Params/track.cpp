/**
 * Husky Model Predictive Contouring Control
 * 
 * Part of Husky Controller Package, visit https://github.com/MURDriverless/HuskyControl for latest version and instructions on how to use
 * This code is heavily based on Alex Liniger's MPCC repo @ https://github.com/alexliniger/MPCC
 * 
 * Current Version by Kheng Yu Yeoh, contact @ khengyu_061192@hotmail.com
 */

#include "track.h"

// Shift start of track to 0, then increase track size by factor k
void MultiplyVector(std::vector<double> &v, double k)
{
    // transform(v.begin(), v.end(), v.begin(), [v,k](double &c){ return (c-v.front())*k; }); // If want to shift start position, e.g. for eufs
    transform(v.begin(), v.end(), v.begin(), [v,k](double &c){ return c*k; });
}

namespace mpcc{

Track::Track() {} ;

Track::Track(std::string file) 
{
    /////////////////////////////////////////////////////
    // Loading Model and Constraint Parameters //////////
    /////////////////////////////////////////////////////
    std::ifstream iTrack(file);
    json jsonTrack;
    iTrack >> jsonTrack;

    const double FACTOR = jsonTrack["Factor"];

    // Model Parameters
    std::vector<double> x = jsonTrack["X"];
    MultiplyVector(x, FACTOR);
    X = Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
    std::vector<double> y = jsonTrack["Y"];
    MultiplyVector(y, FACTOR);
    Y = Eigen::Map<Eigen::VectorXd>(y.data(), y.size());
    
    std::vector<double> x_inner = jsonTrack["X_i"];
    MultiplyVector(x_inner, FACTOR);
    X_inner = Eigen::Map<Eigen::VectorXd>(x_inner.data(), x_inner.size());
    std::vector<double> y_inner = jsonTrack["Y_i"];
    MultiplyVector(y_inner, FACTOR);
    Y_inner = Eigen::Map<Eigen::VectorXd>(y_inner.data(), y_inner.size());

    std::vector<double> x_outer = jsonTrack["X_o"];
    MultiplyVector(x_outer, FACTOR);
    X_outer = Eigen::Map<Eigen::VectorXd>(x_outer.data(), x_outer.size());
    std::vector<double> y_outer = jsonTrack["Y_o"];
    MultiplyVector(y_outer, FACTOR);
    Y_outer = Eigen::Map<Eigen::VectorXd>(y_outer.data(), y_outer.size());

    // Write current track into csv
    std::ofstream trackfile;
    trackfile.open("/home/khengyu/catkin_ws/src/HuskyControl/husky_mpcc/src/Params/track.csv");
    trackfile << "x_o,y_o,x_i,y_i,x,y\n";
    for (int i = 0; i < x.size(); i++)
    {
        if (i > x_outer.size())
        {
            trackfile << "0,0,";
        }
        else
        {
            trackfile << x_outer[i] << "," << y_outer[i] << ",";
        }

        if (i > x_inner.size())
        {
            trackfile << "0,0,";
        }
        else
        {
            trackfile << x_inner[i] << "," << y_inner[i] << ",";
        }

        trackfile << x[i] << "," << y[i] << ",\n";
    }
    
    trackfile.close();
}

Track::Track(std::vector<double> x_outer, std::vector<double> y_outer, 
                        std::vector<double> x_inner, std::vector<double> y_inner, 
                        std::vector<double> x_centre, std::vector<double> y_centre) 
{
    X_outer = Eigen::Map<Eigen::VectorXd>(x_outer.data(), x_outer.size());
    Y_outer = Eigen::Map<Eigen::VectorXd>(y_outer.data(), y_outer.size());
    X_inner = Eigen::Map<Eigen::VectorXd>(x_inner.data(), x_inner.size());
    Y_inner = Eigen::Map<Eigen::VectorXd>(y_inner.data(), y_inner.size());
    X = Eigen::Map<Eigen::VectorXd>(x_centre.data(), x_centre.size());
    Y = Eigen::Map<Eigen::VectorXd>(y_centre.data(), y_centre.size());

    // CubicSpline2D outer = CubicSpline2D(X_outer, Y_outer, 3.0);
    // CubicSpline2D inner = CubicSpline2D(X_inner, Y_inner, 3.0);
    // CubicSpline2D centre = CubicSpline2D(X_centre, Y_centre, 3.0);
    // CubicSpline2D path = CubicSpline2D(X_centre, Y_centre, 3.0);

    // return { outer, inner, centre, path };
}

TrackPos Track::getTrack()
{
    return {1.0*Eigen::Map<Eigen::VectorXd>(X.data(), X.size()), 1.0*Eigen::Map<Eigen::VectorXd>(Y.data(), Y.size()),
            1.0*Eigen::Map<Eigen::VectorXd>(X_inner.data(), X_inner.size()), 1.0*Eigen::Map<Eigen::VectorXd>(Y_inner.data(), Y_inner.size()),
            1.0*Eigen::Map<Eigen::VectorXd>(X_outer.data(), X_outer.size()), 1.0*Eigen::Map<Eigen::VectorXd>(Y_outer.data(), Y_outer.size())};
}

}
