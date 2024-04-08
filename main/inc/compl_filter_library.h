#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        double x;
        double y;
        double z;
    } DataIMU;

    typedef struct
    {
        double pitch;
        double roll;
        double yaw;
    } EulerEstimation;

    typedef struct
    {
        double right_wheels;
        double left_wheels;
    } VelocityWheels;

    typedef struct
    {
        double right_rear;
        double right_front;
        double left_rear;
        double left_front;
    } Wheels;

    typedef struct
    {
        double x;
        double y;
    } Position;

    double integration_trapezoidal_rule(double prev_output_value, double current_value, double prev_value, float d_time);
    double integration_riemann(double prev_output_value, double current_value, float d_time);
    double calculate_yaw_imu(DataIMU angular_velocity, double prev_yaw, double d_time);
    double calculate_yaw_wheels(VelocityWheels velocity_wheels, double current_value, double wheels_distance, double d_time);
    VelocityWheels calculate_velocities_wheels(Wheels despl_linear, double d_time);
    Position calculate_position_wheels(VelocityWheels v_wheels, Position prev_position, float d_time, float yaw);
    Position calculate_position_imu(DataIMU l_a_imu, DataIMU *prev_vel_imu, Position prev_position, float d_time, float yaw);
    Position get_position_f_c(Position imu_position_estime, Position wheels_position_estime, float gamma, float beta);
    double get_orientation_f_c(double imu_orientation_estime, double wheels_orientation_estime, float alpha);

#ifdef __cplusplus
}
#endif
