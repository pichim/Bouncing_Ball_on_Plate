#include "IMU.h"

IMU::IMU(PinName pin_sda,
         PinName pin_scl) : m_i2c(pin_sda, pin_scl),
                            m_ImuMPU6500(m_i2c),
                            m_Mahony(BBOP_IMU_KP, BBOP_IMU_KI, static_cast<float>(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US) * 1.0e-6f)

{
    const float Ts = static_cast<float>(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US) * 1.0e-6f;

    m_gyro_filter[0].lowPass1Init(BBOP_IMU_GYRO_FILTER_FREQUENCY_HZ, Ts);
    m_gyro_filter[1].lowPass1Init(BBOP_IMU_GYRO_FILTER_FREQUENCY_HZ, Ts);
    m_gyro_filter[2].lowPass1Init(BBOP_IMU_GYRO_FILTER_FREQUENCY_HZ, Ts);

    m_acc_filter[0].lowPass1Init(BBOP_IMU_ACC_FILTER_FREQUENCY_HZ, Ts);
    m_acc_filter[1].lowPass1Init(BBOP_IMU_ACC_FILTER_FREQUENCY_HZ, Ts);
    m_acc_filter[2].lowPass1Init(BBOP_IMU_ACC_FILTER_FREQUENCY_HZ, Ts);

    m_gyro_offset.setZero();
    m_acc_offset.setZero();

    m_ImuMPU6500.init();
    m_ImuMPU6500.configuration();
    m_ImuMPU6500.testConnection();
}

IMU::ImuData IMU::getImuData()
{
    // update imu
    m_ImuMPU6500.readGyroAll();
    m_ImuMPU6500.readAccAll();

    // skip first Nskip runs
    if (m_skip_cntr++ < BBOP_IMU_NUM_RUNS_SKIP)
        return m_ImuData;

    Eigen::Vector3f gyro(m_ImuMPU6500.getGyroX(), m_ImuMPU6500.getGyroY(), m_ImuMPU6500.getGyroZ());
    Eigen::Vector3f acc(m_ImuMPU6500.getAccX(), m_ImuMPU6500.getAccY(), m_ImuMPU6500.getAccZ());

    if (!m_is_calibrated) {

        m_avg_cntr++;

        // sum up gyro and acc
        m_gyro_offset += gyro;
        m_acc_offset += acc;

        // calculate average
        if (m_avg_cntr == BBOP_IMU_NUM_RUNS_FOR_AVERAGE) {
            m_is_calibrated = true;

            m_gyro_offset /= m_avg_cntr;
            m_acc_offset /= m_avg_cntr;

            printf("IMU calibrated.\n");
            printf("Avg. Gyr offset: %.4f, %.4f, %.4f; ...\n", m_gyro_offset(0), m_gyro_offset(1), m_gyro_offset(2));
            printf("Avg. Acc offset: %.4f, %.4f, %.4f; ...\n", m_acc_offset(0), m_acc_offset(1), m_acc_offset(2));

#if BBOP_IMU_DO_USE_STATIC_ACC_CALIBRATION
            m_acc_offset = BBOP_IMU_B_ACC;
#endif
        }
    } else {
        // remove static bias
        gyro -= m_gyro_offset;
        acc -= m_acc_offset;

#if BBOP_IMU_USE_ADDITIONAL_FILTERS
        if (m_is_first_run) {
            m_is_first_run = false;
            for (uint8_t i = 0; i < 3; i++) {
                m_gyro_filter[i].reset(gyro(i));
                m_acc_filter[i].reset(acc(i));
            }
        }
        // filter gyro and acc data
        for (uint8_t i = 0; i < 3; i++) {
            gyro(i) = m_gyro_filter[i].apply(gyro(i));
            acc(i) = m_acc_filter[i].apply(acc(i));
        }
#endif

        // update mahony
        m_Mahony.update(gyro, acc);

        // update data object
        m_ImuData.gyro = gyro;
        m_ImuData.acc = acc;
        m_ImuData.quat = m_Mahony.getOrientationAsQuaternion();
        m_ImuData.rpy = m_Mahony.getOrientationAsRPYAngles();
        m_ImuData.tilt = m_Mahony.getTiltAngle();
    }

    return m_ImuData;
}
