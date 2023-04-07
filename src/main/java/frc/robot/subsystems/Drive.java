package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class Drive extends SubsystemBase{
    // Gyro
    private WPI_Pigeon2 gyro_;
    
    // Create instances for each Swerve Module
    private final SwerveModule frontLeft_ = new SwerveModule(Constants.kFrontLeftDriveMotor, Constants.kFrontLeftSteerMotor, Constants.kFrontLeftCANCoder);
    private final SwerveModule frontRight_ = new SwerveModule(Constants.kFrontRightDriveMotor, Constants.kFrontRightSteerMotor, Constants.kFrontRightCANCoder);
    private final SwerveModule backLeft_ = new SwerveModule(Constants.kBackLeftDriveMotor, Constants.kBackLeftSteerMotor, Constants.kBackLeftCANCoder);
    private final SwerveModule backRight_ = new SwerveModule(Constants.kBackRightDriveMotor, Constants.kBackRightSteerMotor, Constants.kBackRightCANCoder);

    
    // All Translation2d(x,y) values need to be updated to the drivetrain
    // center to each swerve module (meters)
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); 
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // creating kinematics object
    SwerveDriveKinematics swerveKinematics_ = new SwerveDriveKinematics(
        m_frontLeftLocation, 
        m_frontRightLocation, 
        m_backLeftLocation, 
        m_backRightLocation
    );

    // Check if this works
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(frontLeft_.getDrivePosition(), Rotation2d.fromDegrees(frontLeft_.getSteerPosition())),
        new SwerveModulePosition(frontRight_.getDrivePosition(), Rotation2d.fromDegrees(frontRight_.getSteerPosition())),
        new SwerveModulePosition(backLeft_.getDrivePosition(), Rotation2d.fromDegrees(backLeft_.getSteerPosition())),
        new SwerveModulePosition(backRight_.getDrivePosition(), Rotation2d.fromDegrees(backRight_.getSteerPosition())),
    };

    SwerveDriveOdometry odometer = new SwerveDriveOdometry(swerveKinematics_, getRotation2d(), modulePositions);

    // Constructor
    public Drive() {
        // Initialize Gyro
        gyro_ = new WPI_Pigeon2(Constants.kGyroId);
        gyro_.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);

        // Delays the zeroing of the gyro but allows other stuff to continue happening
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    // Methods
    public void zeroHeading() {
        gyro_.reset();
    }

    public double getYaw() {
        return Math.toRadians(gyro_.getYaw());
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), modulePositions, pose);
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveKinematics_;
    }
    
    // Other odometry stuff -- fix the position stuff first

    public void stopModules() {
        frontLeft_.stop();
        frontRight_.stop();
        backLeft_.stop();
        backRight_.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        /* 
        incase any module is given a speed that is higher than the max, 
        desaturateWheelSpeeds() will reduce all of the module speeds until all module speeds are under the limit 
        */
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond); 

        frontLeft_.setDesiredState(desiredStates[0]);
        frontRight_.setDesiredState(desiredStates[1]);
        backLeft_.setDesiredState(desiredStates[2]);
        backRight_.setDesiredState(desiredStates[3]);
    }
    
    // Constants Class
    public static class Constants {
        // Drive Motor IDs
        public static final int kFrontLeftDriveMotor = 0;
        public static final int kFrontRightDriveMotor = 0;
        public static final int kBackLeftDriveMotor = 0;
        public static final int kBackRightDriveMotor = 0;

        // Steer Motor IDs
        public static final int kFrontLeftSteerMotor = 0;
        public static final int kFrontRightSteerMotor = 0;
        public static final int kBackLeftSteerMotor = 0;
        public static final int kBackRightSteerMotor = 0;

        // CANCoder IDs
        public static final int kFrontLeftCANCoder = 0;
        public static final int kFrontRightCANCoder = 0;
        public static final int kBackLeftCANCoder = 0;
        public static final int kBackRightCANCoder = 0;

        // Gyro ID
        public static final int kGyroId = 0;

        // Values :)
        public static final int kPhysicalMaxSpeedMetersPerSecond = 1;
    }
}
