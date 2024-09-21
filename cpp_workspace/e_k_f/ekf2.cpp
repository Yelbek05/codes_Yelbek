#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

class KalmanFilter
{
public:

    using Matrix3x2 = Eigen::Matrix<double, 3, 2>; ///<

    KalmanFilter() : 
        m_dt{0.1},
        m_state{46.88062202333334, 16.840152835, 186.60000610351562},
        m_A{Eigen::Matrix3d::Identity()},
        m_P{0.1 * Eigen::Matrix3d::Identity()},
        m_P_old{0.1 * Eigen::Matrix3d::Identity()},
        m_Q{1.0 * Eigen::Matrix3d::Identity()},
        m_H{1.0 * Eigen::Matrix3d::Identity()},
        m_R{1.0 * Eigen::Matrix3d::Identity()}
    {
        m_B = getB(m_state.z(), m_dt);
    }

    Matrix3x2 getB(double theta, double dt)
    {
        Matrix3x2 B;
        B << std::cos(theta) * dt, 0,
             std::sin(theta) * dt, 0,
             0, dt;
        return B;
    }

    void predict(Eigen::Vector2d input = Eigen::Vector2d(0.0, 0.0))
    {
        m_B = getB(m_state.z(), m_dt);
        m_state = m_A * m_state + m_B * input;
        // m_P (k-1) | (k-1)
        m_P = m_A * m_P * m_A.transpose() + m_Q;
        // m_P (k) | (k-1)
    }

    void update(const Eigen::Vector3d& measurement)
    {
        // m_P (k) | (k-1)
        S_k = m_H * m_P * m_H.transpose() + m_R; 
        K_k = m_P_old * m_H.transpose() * S_k.inverse();

        m_state = m_state + K_k * (measurement - m_H * m_state);
        // m_P (k) | (k-1)
        m_P = (Eigen::Matrix3d::Identity() - K_k * m_H) * m_P;  
        // m_P (k) | (k)
        m_P_old = m_P;
    }

    const Eigen::Vector3d& getState() const { return m_state; }

private:
    double m_dt;
    Eigen::Vector3d m_state; ///< x, y, theta
    Eigen::Matrix3d m_A; ///< 
    Matrix3x2 m_B; ///< 

    Eigen::Matrix3d m_P;
    Eigen::Matrix3d m_P_old;
    Eigen::Matrix3d m_Q;

    Eigen::Matrix3d m_H;
    Eigen::Matrix3d m_R;

    Eigen::Matrix3d S_k; //Innovation_Covariance
    Eigen::Matrix3d K_k; //Kalman_gain
};

#endif // KALMAN_FILTER_HPP