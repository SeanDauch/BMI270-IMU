#ifndef BMI270_drivers_h
    #define BMI270_drivers_h

    #include <stdint.h>
    enum init_status{
        OK,
        COMMUNICATION_ERROR,
        INITIALIZATION_ERROR
    };

    typedef struct {
        uint16_t acc_sensitivity;
        uint16_t acc_frequency;
        float gyro_sensitivity;
        uint16_t gyro_frequency;
    }BMI_settings;

    enum init_status BMI270_init(BMI_settings* BMI_settings);
    void enable_data_ready_interrupt();

    struct data_3D{
        float x;
        float y;
        float z;
    };
    struct data_3D get_accel_data(BMI_settings* settings);
    struct data_3D get_gyro_data(BMI_settings* settings);
    float get_pitch_from_accel(struct data_3D* accel_data);
    float get_roll_from_accel(struct data_3D* accel_data);
    float integrate_gyro(BMI_settings* settings, float one_axis_gyro_data);
#endif