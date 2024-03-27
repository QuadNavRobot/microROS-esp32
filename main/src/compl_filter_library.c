#include "../inc/compl_filter_library.h"

/**
 * @brief Realiza una integración por el método del trapecio.
 * @param prev_output_value Valor acumulado anterior de la integral.
 * @param current_value Valor del punto actual al que se va a integrar (n).
 * @param prev_value Valor del punto anterior a la integral (n + 1).
 * @param d_time Tiempo transcurrido entre prev_value y current_value.
 * @return Valor acumulado de la integral.
 */
double integration_trapezoidal_rule(double prev_output_value,
                                    double current_value,
                                    double prev_value,
                                    float d_time)
{
    return prev_output_value + (current_value + prev_value) / 2 * d_time;
}

/**
 * @brief Realiza una integración por el método de Riemann.
 * @param prev_output_value Valor acumulado anterior de la integral.
 * @param current_value Valor del punto actual al que se va a integrar (n).
 * @param d_time Tiempo transcurrido entre current_value y el punto anterior
 * calculado.
 * @return Valor acumulado de la integral.
 */
double integration_riemann(double prev_output_value,
                           double current_value,
                           float d_time)
{
    return prev_output_value + current_value * d_time;
}

/**
 * @brief Calculo de la estimación del angulo yaw, formado sobre el plano x,y, con los datos del giroscopio.
 * @param angular_velocity Dato obtenido del giroscopio.
 * @param prev_yaw Estimación previa de yaw.
 * @param d_time Tiempo transcurrido desde el último dato obtenido.
 * @return Estimación del ángulo, yaw, formado sobre el plano x,y.
 */
double calculate_yaw_imu(DataIMU angular_velocity, double prev_yaw, double d_time)
{
    return angular_velocity.z * d_time + prev_yaw;
}

/**
 * @brief Calculo de la estimación del angulo yaw, formado sobre el plano x,y, con los datos de las velocidades de las ruedas.
 * @param v_wheels Velocidades lineales estimadas de las ruedas.
 * @param current_value Estimación actual del angulo yaw.
 * @param wheels_distance Distancia entre las ruedas derechas e izquerdas.
 * @param d_time Tiempo transcurrido desde el último dato obtenido.
 * @return Estimación del ángulo, yaw, formado sobre el plano x,y.
 */
double calculate_yaw_wheels(VelocityWheels velocity_wheels, double current_value, double wheels_distance, double d_time)
{
    float despl_angular = (velocity_wheels.right_wheels - velocity_wheels.left_wheels) / wheels_distance;

    return integration_riemann(current_value, despl_angular, d_time);
}

/**
 * @brief Calculo de las velocidades angulares de las ruedas derechas eh izquierda.
 * @param angle_wheels Ángulos actuales de cada rueda respecto al chasis.
 * @param times_wheels Tiempo transcurrido desde la ultima medición de las ruedas.
 * @return Velocidades angulares del promedio de las ruedas derechas eh izquierda.
 */
VelocityWheels calculate_velocities_wheels(Wheels angle_wheels, double times_wheels)
{
    double vel_w[4];

    double *point_angles = &angle_wheels.right_rear;
    for (int i = 0; i < 4; i++)
    {
        vel_w[i] = point_angles[i] / times_wheels;
    }
    // Calculo el promedio de las velocidades de las ruedas de cada lado del vehículo
    VelocityWheels velocities = {(vel_w[0] + vel_w[1]) / 2,
                                 (vel_w[2] + vel_w[3]) / 2};

    return velocities;
}

/**
 * @brief Estimación de la posición del robot con la velocidad de las ruedas.
 * @param v_wheels Velocidades lineales estimadas de las ruedas.
 * @param prev_position Posición estimada anterior.
 * @param d_time Tiempo transcurrido desde la estimación anterior a la actual.
 * @param yaw Ángulo estimado de desplazamiento angular del vehículo en el lapso d_time.
 * @return Estimación de posición actual.
 */
Position calculate_position_wheels(VelocityWheels v_wheels, Position prev_position, float d_time, float yaw)
{
    Position position;
    float v = (v_wheels.left_wheels + v_wheels.right_wheels) / 2;
    position.x = integration_riemann(prev_position.x, v * cos(yaw), d_time);
    position.y = integration_riemann(prev_position.y, v * sin(yaw), d_time);

    return position;
}

/**
 * @brief Estimación de la posición del robot con los datos obtenidos del acelerómetro.
 * @param l_a_imu Aceleraciones lineales obtenidas por el acelerómetro.
 * @param prev_vel_imu Velocidades previas estimadas, se actualizarán con las nuevas estimaciones.
 * @param prev_position Posición estimada anterior.
 * @param d_time Tiempo transcurrido desde la estimación anterior a la actual.
 * @param yaw Ángulo estimado de desplazamiento angular del vehículo en el lapso d_time.
 * @return Estimación de posición actual.
 */
Position calculate_position_imu(DataIMU l_a_imu, DataIMU *prev_vel_imu, Position prev_position, float d_time, float yaw)
{
    Position position;
    double v_x = integration_riemann(prev_vel_imu->x, l_a_imu.x, d_time);
    double v_y = integration_riemann(prev_vel_imu->y, l_a_imu.y, d_time);

    position.x = integration_riemann(prev_position.x, v_x * cos(yaw), d_time);
    position.y = integration_riemann(prev_position.y, v_y * sin(yaw), d_time);

    prev_vel_imu->x = v_x;
    prev_vel_imu->y = v_y;

    return position;
}

/**
 * @brief Estimación de la posición del robot con la técnica del filtro complementario
 * utilizando las estimaciones obtenidas de la IMU y de los encoders.
 * @param imu_position_estime Estimación actual de la posición del robot calculado con la imu.
 * @param wheels_position_estime Estimación actual de la posición del robot calculado con los encoders.
 * @param gamma Ponderación de valores para la posición x, donde mientras mayor sea el valor de gamma,
 * mayor "confianza" se tiene sobre los valores obtenidos de las ruedas.
 * @param beta Ponderación de valores para la posición y, donde mientras mayor sea el valor de beta,
 * mayor "confianza" se tiene sobre los valores obtenidos de las ruedas.
 * @return Nueva estimación de la posición del robot.
 */
Position get_position_f_c(Position imu_position_estime,
                               Position wheels_position_estime,
                               float beta,
							   float gamma)
{
    Position position;

    position.x = wheels_position_estime.x * gamma + imu_position_estime.x * (1 - gamma);
    position.y = wheels_position_estime.y * beta + imu_position_estime.y * (1 - beta);

    return position;
}

/**
 * @brief Estimación de la orientación del robot con la técnica del filtro complementario
 * utilizando las estimaciones obtenidas de la IMU y de los encoders.
 * @param imu_orientation_estime Estimación actual de la orientación del robot calculados con la imu.
 * @param wheels_orientation_estime Estimación actual de la orientación del robot calculados con los encoders.
 * @param alpha Ponderación de valores, donde mientras mayor sea el valor de alpha,  mayor "confianza"
 * se tiene sobre los valores obtenidos de las ruedas.
 */
double get_orientation_f_c(double imu_orientation_estime,
                                double wheels_orientation_estime,
                                float alpha)
{
    double orientation;

    orientation = wheels_orientation_estime * alpha + imu_orientation_estime * (1 - alpha);

    return orientation;
}
