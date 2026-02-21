"""
host — ICM-42688-P Velocity-Based Training (VBT) pipeline

Modules
-------
imu_driver     Packet decode from FPGA UART
calibration    Static bias estimation at startup
estimator      EKF orientation + velocity/position tracking
vbt            Main VBT application
"""
