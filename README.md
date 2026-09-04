### Setting baremetal drivers up for this IMU

### Materials:
    STM32F411
    BMI270: https://www.digikey.com/en/products/detail/bosch-sensortec/BMI270/9974486

### Next Steps:
1. ~~Turn sensor numbers to real measurements~~
2. ~~Obtain pitch/roll from accelerometer~~
3. ~~Implement Kalman/Madgwick/Complementary filter~~ (Complementary Filter)
4. ~~Figure out real-time graphing to test filters~~ (STM32CubeMonitor)
5. ~~Add EXTI for accurate gyro measurement~~

### What I Learned:
1. Avoid doubles
    - FPU doesnt treat floats and doubles the same so it can 50-100x cycles per operation
2. Sensor fusion
    - I only used a complementary filter but I learned about Madgwick/Kalman filters and quaternions
3. EXTI
    - I had been meaning to learn about interrupts and this was a good intro
4. STM32CubeMonitor
    - Super duper helpful for debugging and real-time plotting is invaluable
