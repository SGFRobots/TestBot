package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;


public class SwerveSubsystem {
    private final SwerveModule frontLeft = new ServeModule(
        Constants.kFrontLeftDriveMotorPort,
        Constants.kFrontLeftTurningMotorPort,
        Constants.kFrontLeftDriveEncoderReversed,
        Constants.kFrontLeftTurningEncoderReversed,
        Constants.kFrontLeftDriveAbsoluteEncoderPort,
        Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        Constants.kFrontLeftDriveAbsoluteEncoderReversed);
    
    private final SwerveModule frontRight = new ServeModule(
        Constants.kFrontRightDriveMotorPort,
        Constants.kFrontRightTurningMotorPort,
        Constants.kFrontRightDriveEncoderReversed,
        Constants.kFrontRightTurningEncoderReversed,
        Constants.kFrontRightDriveAbsoluteEncoderPort,
        Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        Constants.kFrontRightDriveAbsoluteEncoderReversed);
    
    private final SwerveModule backLeft = new ServeModule(
        Constants.kBackLeftDriveMotorPort,
        Constants.kBackLeftTurningMotorPort,
        Constants.kBackLeftDriveEncoderReversed,
        Constants.kBackLeftTurningEncoderReversed,
        Constants.kBackLeftDriveAbsoluteEncoderPort,
        Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        Constants.kBackLeftDriveAbsoluteEncoderReversed);
    
    private final SwerveModule backRight = new ServeModule(
        Constants.kBackRightDriveMotorPort,
        Constants.kBackRightTurningMotorPort,
        Constants.kBackRightDriveEncoderReversed,
        Constants.kBackRightTurningEncoderReversed,
        Constants.kBackRightDriveAbsoluteEncoderPort,
        Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
        Constants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics,
            new Rotation2d(0));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Roataion2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), 
        backRight.getState())
        SmartDashboard.putNumber('Robot Heading', getHeading());
        SmartDashboard.putNumber('Robot Location', getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
