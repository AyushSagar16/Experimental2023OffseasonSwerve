package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {
    // Motor Controllers
    private final CANSparkMax drive_motor_;
    private final CANSparkMax steer_motor_;

    // Sensors
    private final RelativeEncoder drive_encoder_;
    private final RelativeEncoder steer_encoder_;
    private final CANCoder can_coder_;

    // Control
    private final PIDController drive_pid_controller_; // in m/s
    private final PIDController steer_pid_controller_; // in radians

    // Constructor
    public SwerveModule(int kDriveMotorID, int kSteerMotorID, int kCanCoderID){
        // Initialize motor controllers
        drive_motor_ = new CANSparkMax(kDriveMotorID, CANSparkMax.MotorType.kBrushless);
        steer_motor_ = new CANSparkMax(kSteerMotorID, CANSparkMax.MotorType.kBrushless);

        // Initialize encoders
        drive_encoder_ = drive_motor_.getEncoder();
        drive_encoder_.setPositionConversionFactor(2 * Math.PI * Constants.kWheelRadius / Constants.kDriveGearRatio);
        drive_encoder_.setVelocityConversionFactor(2 * Math.PI * Constants.kWheelRadius / Constants.kDriveGearRatio / 60);

        steer_encoder_ = steer_motor_.getEncoder();
        steer_encoder_.setPositionConversionFactor(2 * Math.PI * Constants.kSteerGearRatio);
        steer_encoder_.setVelocityConversionFactor(2 * Math.PI * Constants.kSteerGearRatio / 60);
        
        can_coder_ = new CANCoder(kCanCoderID);
        CANCoderConfiguration config = new CANCoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        can_coder_.configAllSettings(config);

        // Initialize PID controllers
        drive_pid_controller_ = new PIDController(Constants.kDriveKp, 0.0, 0.0);
        
        steer_pid_controller_ = new PIDController(Constants.kSteerKp, 0.0, 0.0);
        steer_pid_controller_.enableContinuousInput(-Math.PI, Math.PI);

        // Reset encoders
        resetEncoders();
    }

    // Methods
    public double getDrivePosition(){
        // Need to set up the factor for the encoder later
        // ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRot2Meter)
        return drive_encoder_.getPosition();
    }

    public double getSteerPosition(){
        return steer_encoder_.getPosition();
    }

    public double getDriveVelocity(){
        // ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRPM2MeterPerSec)
        return drive_encoder_.getVelocity();
    }

    public double getSteerVelocity(){
        return steer_encoder_.getVelocity();
    }

    public double getCANCoderRad(){
        return can_coder_.getAbsolutePosition();
    }

    public void resetEncoders(){
        drive_encoder_.setPosition(0);
        steer_encoder_.setPosition(getCANCoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getState().angle);

        double desiredDrive = state.speedMetersPerSecond / Constants.kMaxSpeed;

        drive_motor_.set(drive_pid_controller_.calculate(getDriveVelocity(), desiredDrive));
        steer_motor_.set(steer_pid_controller_.calculate(getSteerPosition(), state.angle.getRadians()));
    }

    public void stop() {
        drive_motor_.set(0);
        steer_motor_.set(0);
    }

    // Constants Class
    public static class Constants{
        // Control *Change after tuning is done*
        public static final double kDriveKp = 1.0;
        public static final double kSteerKp = 1.0;

        // Hardware *Change after building is done*
        public static double kDriveGearRatio = 10.18;
        public static double kSteerGearRatio = 10.18;
        public static double kWheelRadius = 0.0762;

        // Limits
        public static final double kMaxSpeed = 3.0; // m/s
    }
}
