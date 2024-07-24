package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.Constants;


public class SwerveSubsystem extends SubsystemBase{
    // Make instances of all 4 modules
    private final SwerveModule[] modules = {
        // Front Left
        new SwerveModule(
            Constants.kFrontLeftDriveMotorPort,
            Constants.kFrontLeftTurningMotorPort,
            Constants.kFrontLeftDriveEncoderReversed,
            Constants.kFrontLeftTurningEncoderReversed,
            Constants.kFrontLeftDriveAbsoluteEncoderPort,
            Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontLeftDriveAbsoluteEncoderReversed),
        
        // Front Right
        new SwerveModule(
            Constants.kFrontRightDriveMotorPort,
            Constants.kFrontRightTurningMotorPort,
            Constants.kFrontRightDriveEncoderReversed,
            Constants.kFrontRightTurningEncoderReversed,
            Constants.kFrontRightDriveAbsoluteEncoderPort,
            Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontRightDriveAbsoluteEncoderReversed),
        
        // Back Left
        new SwerveModule(
            Constants.kBackLeftDriveMotorPort,
            Constants.kBackLeftTurningMotorPort,
            Constants.kBackLeftDriveEncoderReversed,
            Constants.kBackLeftTurningEncoderReversed,
            Constants.kBackLeftDriveAbsoluteEncoderPort,
            Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kBackLeftDriveAbsoluteEncoderReversed),
        
        // Back Right
        new SwerveModule(
            Constants.kBackRightDriveMotorPort,
            Constants.kBackRightTurningMotorPort,
            Constants.kBackRightDriveEncoderReversed,
            Constants.kBackRightTurningEncoderReversed,
            Constants.kBackRightDriveAbsoluteEncoderPort,
            Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
            Constants.kBackRightDriveAbsoluteEncoderReversed),
        
    };

    // Position stored in gyro and odometer
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics,
        new Rotation2d(0), 
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        }, new Pose2d(5.0, 13.5, new Rotation2d()));

    // =)
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                // Reset position
                gyro.reset();
            } catch (Exception e) {
            }
        }).start();
    }

    // Get angle robot is facing
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    // Get direction robot is facing
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Get position of robot
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        // update position
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()});
        // Update distances for all modules
        for (SwerveModule module : modules) {
            module.updateDistance();
        }

        // Debug telemetry
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    // Stop the robot
    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stop();
        }
    }

    // DRIVE the robot
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Set speed to max if go over max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        // Move modules
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }
}
