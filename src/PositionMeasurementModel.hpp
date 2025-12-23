#ifndef KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_

#include <cmath>
#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"




namespace Robot1
{


template<typename T>
class PositionMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(PositionMeasurement, T, 1)
    
    //! Distance to landmark 1
    static constexpr size_t DEL_T = 0;
    


    
  T del_t()       const { return (*this)[ DEL_T ]; }

    
  T& del_t()      { return (*this)[ DEL_T ]; }

};


template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class PositionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  Robot1::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  Robot1::PositionMeasurement<T> M;

    mutable float h_pos;
    mutable float k_pos;
    mutable float a;
    mutable float b;
    mutable float d;    


    PositionMeasurementModel(T h_, T k_, T theta, T d_)
      : h_pos(h_), k_pos(k_), a(std::cos(theta)), b(std::sin(theta)), d(d_)        
    {   
        this->V.setIdentity();
    }



    M h(const S& x) const        
    {
        M measurement;


        float vec_i = x.x() - h_pos;
        float vec_j = x.y() - k_pos;

        float magn = std::sqrt((vec_i * vec_i) + (vec_j * vec_j));

	float constant = d / 340.00; // d/v, velosity of sound is 337 in dhulikhel

        float lhs = b * (vec_i / magn);
        float rhs = a * (vec_j / magn);

	measurement.del_t() = constant * (lhs - rhs);

        return measurement;        
    }
    

protected:
    
    
  void updateJacobians(const S &x, const double t = 0.05) {
    
    }
};

} // namespace Robot


#endif
