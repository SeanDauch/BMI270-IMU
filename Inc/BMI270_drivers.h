#ifndef BMI270_drivers_h
    #define BMI270_drivers_h

    enum init_status{
    OK,
    COMMUNICATION_ERROR,
    INITIALIZATION_ERROR
    };
    enum init_status BMI270_init();

    struct data_3D{
    double x;
    double y;
    double z;
    };
    struct data_3D get_accel_data();
    struct data_3D get_gyro_data();
#endif