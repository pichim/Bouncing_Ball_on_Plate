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

    // Acc calibration is fixed, but gyro bias is calibrated at startup
    m_is_calibrated = false;
    m_is_first_run = true;
    m_avg_cntr = 0;
    m_skip_cntr = 0;

    // fixed gyro bias
    // m_gyro_offset << -0.0983f,  0.0913f,  0.0151f;
    m_gyro_offset.setZero();

    // fixed accelerometer offset b
    m_acc_offset <<  0.0585f, -0.0616f, -0.1592f;

    // fixed accelerometer calibration matrix A
    m_acc_A <<
        1.0056f, 0.0f,    0.0f,
        0.0f,    1.0052f, 0.0f,
        0.0f,    0.0f,    0.9893f;

    m_ImuMPU6500.init();
    m_ImuMPU6500.configuration();
    m_ImuMPU6500.testConnection();
}

IMU::ImuData IMU::getImuData()
{
    // Read new IMU values
    m_ImuMPU6500.readGyroAll();
    m_ImuMPU6500.readAccAll();

    // Skip first samples, because the sensor values can be unstable directly after startup
    if (m_skip_cntr++ < BBOP_IMU_NUM_RUNS_SKIP)
        return m_ImuData;

    Eigen::Vector3f gyro(m_ImuMPU6500.getGyroX(), m_ImuMPU6500.getGyroY(), m_ImuMPU6500.getGyroZ());
    Eigen::Vector3f acc(m_ImuMPU6500.getAccX(), m_ImuMPU6500.getAccY(), m_ImuMPU6500.getAccZ());

    // Startup gyro calibration
    // Plate must be standing still during this part
    if (!m_is_calibrated) {

        m_gyro_offset += gyro;
        m_avg_cntr++;

        // calculate average
        if (m_avg_cntr == BBOP_IMU_NUM_RUNS_FOR_AVERAGE) {

            m_gyro_offset /= m_avg_cntr;
            m_is_calibrated = true;
            m_is_first_run = true; // reset filters after calibration

            printf("IMU calibrated.\n");
            printf("Avg. Gyr offset: %.4f, %.4f, %.4f; ...\n", m_gyro_offset(0), m_gyro_offset(1), m_gyro_offset(2));

            }

        // During calibration, return old data.
        // The controller should not actively move the plate during this time.
        return m_ImuData;    

    } 

    // Apply calibration
    gyro -= m_gyro_offset;
    acc   = m_acc_A * (acc - m_acc_offset);

#if BBOP_IMU_USE_ADDITIONAL_FILTERS
    
    // Reset filters with first valid calibrated values
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

    // Store IMU data
    m_ImuData.gyro = gyro;
    m_ImuData.acc = acc;
    m_ImuData.quat = m_Mahony.getOrientationAsQuaternion();

    // Raw angles before mechanical offset correction
    Eigen::Vector3f rpy_raw = m_Mahony.getOrientationAsRPYAngles();

///////Temporary debug print for IMU offset tuning/////////////////////////////////    
    static uint32_t imu_print_cntr = 0;
    imu_print_cntr++;

    if (imu_print_cntr >= 500) {   // bei 1 kHz ungefähr alle 0.5 s
        imu_print_cntr = 0;

        printf("IMU RAW: roll = %.3f deg, pitch = %.3f deg\n",
            rpy_raw(0) * BBOP_RAD_TO_DEG,
            rpy_raw(1) * BBOP_RAD_TO_DEG);
    }
///////////////////////////////////////////////////////////////////////////////////

    // Use raw values first
    m_ImuData.rpy = rpy_raw;

    // Offsets korrigieren
    m_ImuData.rpy(0) += (0.0f) * BBOP_DEG_TO_RAD;     // roll
    m_ImuData.rpy(1) += (0.0f) * BBOP_DEG_TO_RAD;     // pitch
 
    m_ImuData.tilt = m_Mahony.getTiltAngle();

    return m_ImuData;
}