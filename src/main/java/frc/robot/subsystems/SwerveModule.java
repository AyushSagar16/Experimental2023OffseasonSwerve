package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class SwerveModule {
    // Motor Controllers
    private final CANSparkMax drive_motor_;
    private final CANSparkMax steer_motor_;

    // Sensors
    private final CANCoder can_coder_;

    // Control
    private SparkMaxPIDController drive_pid_controller_; // in m/s
    private SparkMaxPIDController steer_pid_controller_; // in radians

    // Constructor
    public SwerveModule(int kDriveMotorID, int kSteerMotorID, int kCanCoderID){
        // Initialize motor controllers
        drive_motor_ = new CANSparkMax(kDriveMotorID, CANSparkMax.MotorType.kBrushless);
        steer_motor_ = new CANSparkMax(kSteerMotorID, CANSparkMax.MotorType.kBrushless);

        // Initialize encoders
        can_coder_ = new CANCoder(kCanCoderID);

        // Initialize PID controllers
        drive_pid_controller_ = drive_motor_.getPIDController();
        steer_pid_controller_ = steer_motor_.getPIDController();
    }
}
