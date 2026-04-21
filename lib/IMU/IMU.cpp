#include "IMU.h"

IMU::IMU(PinName pin_sda,
         PinName pin_scl)
    : m_i2c(pin_sda, pin_scl)
    , m_ImuMPU6500(m_i2c)
    , m_Mahony(BBOP_IMU_KP,
               BBOP_IMU_KI,
               static_cast<float>(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US) * 1.0e-6f)
{
    const float Ts = static_cast<float>(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US) * 1.0e-6f;

    m_gyro_filter[0].lowPass1Init(BBOP_IMU_GYRO_FILTER_FREQUENCY_HZ, Ts);
    m_gyro_filter[1].lowPass1Init(BBOP_IMU_GYRO_FILTER_FREQUENCY_HZ, Ts);
    m_gyro_filter[2].lowPass1Init(BBOP_IMU_GYRO_FILTER_FREQUENCY_HZ, Ts);

    m_acc_filter[0].lowPass1Init(BBOP_IMU_ACC_FILTER_FREQUENCY_HZ, Ts);
    m_acc_filter[1].lowPass1Init(BBOP_IMU_ACC_FILTER_FREQUENCY_HZ, Ts);
    m_acc_filter[2].lowPass1Init(BBOP_IMU_ACC_FILTER_FREQUENCY_HZ, Ts);

    m_gyro_offset << -0.0983f,  0.0913f,  0.0151f;
    m_acc_offset  <<  0.0585f, -0.0616f, -0.1592f;

    m_is_calibrated = true;

    m_ImuMPU6500.init();
    m_ImuMPU6500.configuration();
    m_ImuMPU6500.testConnection();
}

void IMU::getRawData(Eigen::Vector3f& gyro_raw, Eigen::Vector3f& acc_raw)
{
    m_ImuMPU6500.readGyroAll();
    m_ImuMPU6500.readAccAll();

    gyro_raw = Eigen::Vector3f(
        m_ImuMPU6500.getGyroX(),
        m_ImuMPU6500.getGyroY(),
        m_ImuMPU6500.getGyroZ());

    acc_raw = Eigen::Vector3f(
        m_ImuMPU6500.getAccX(),
        m_ImuMPU6500.getAccY(),
        m_ImuMPU6500.getAccZ());
}

IMU::ImuData IMU::getImuData()
{
    m_ImuMPU6500.readGyroAll();
    m_ImuMPU6500.readAccAll();

    if (m_skip_cntr++ < BBOP_IMU_NUM_RUNS_SKIP)
        return m_ImuData;

    Eigen::Vector3f gyro(m_ImuMPU6500.getGyroX(),
                         m_ImuMPU6500.getGyroY(),
                         m_ImuMPU6500.getGyroZ());

    Eigen::Vector3f acc(m_ImuMPU6500.getAccX(),
                        m_ImuMPU6500.getAccY(),
                        m_ImuMPU6500.getAccZ());

    if (!m_is_calibrated) {

        m_avg_cntr++;

        m_gyro_offset += gyro;
        m_acc_offset += acc;

        if (m_avg_cntr == BBOP_IMU_NUM_RUNS_FOR_AVERAGE) {
            m_is_calibrated = true;

            m_gyro_offset /= m_avg_cntr;
            m_acc_offset /= m_avg_cntr;

            m_acc_offset(2) = 0.0f;

            printf("IMU calibrated.\n");
            printf("Avg. Gyr offset: %.4f, %.4f, %.4f\n",
                   m_gyro_offset(0), m_gyro_offset(1), m_gyro_offset(2));
            printf("Avg. Acc offset: %.4f, %.4f, %.4f\n",
                   m_acc_offset(0), m_acc_offset(1), m_acc_offset(2));

#if BBOP_IMU_DO_USE_STATIC_ACC_CALIBRATION
            m_acc_offset = BBOP_IMU_B_ACC;
#endif
        }
    } else {
        gyro -= m_gyro_offset;
        acc  -= m_acc_offset;

        // static accelerometer scaling from MATLAB calibration
        acc(0) *= 1.0056f;
        acc(1) *= 1.0052f;
        acc(2) *= 0.9893f;

#if BBOP_IMU_USE_ADDITIONAL_FILTERS
        if (m_is_first_run) {
            m_is_first_run = false;
            for (uint8_t i = 0; i < 3; i++) {
                m_gyro_filter[i].reset(gyro(i));
                m_acc_filter[i].reset(acc(i));
            }
        }

        for (uint8_t i = 0; i < 3; i++) {
            gyro(i) = m_gyro_filter[i].apply(gyro(i));
            acc(i)  = m_acc_filter[i].apply(acc(i));
        }
#endif

        m_Mahony.update(gyro, acc);

        m_ImuData.gyro = gyro;
        m_ImuData.acc  = acc;
        m_ImuData.quat = m_Mahony.getOrientationAsQuaternion();
        m_ImuData.rpy  = m_Mahony.getOrientationAsRPYAngles();
        m_ImuData.tilt = m_Mahony.getTiltAngle();
    }

    return m_ImuData;
}