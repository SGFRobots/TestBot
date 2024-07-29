// package frc.robot.subsystems.Swerve;

// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;


// public class SwerveSubsystem extends SubsystemBase{
//     // Make instances of all 4 modules
//     private final SwerveModule[] modules = {
//         // Front Left
//         new SwerveModule(
//             Constants.Swerve.MotorPorts.kFrontLeftDriveMotorPort,
//             Constants.Swerve.MotorPorts.kFrontLeftTurningMotorPort,
//             Constants.Swerve.Reversed.kFrontLeftDriveEncoderReversed,
//             Constants.Swerve.Reversed.kFrontLeftTurningEncoderReversed,
//             Constants.Swerve.MotorPorts.kFrontLeftDriveAbsoluteEncoderPort,
//             Constants.Swerve.Mechanical.kFrontLeftDriveAbsoluteEncoderOffsetRad,
//             Constants.Swerve.Reversed.kFrontLeftDriveAbsoluteEncoderReversed),
        
//         // Front Right
//         new SwerveModule(
//             Constants.Swerve.MotorPorts.kFrontRightDriveMotorPort,
//             Constants.Swerve.MotorPorts.kFrontRightTurningMotorPort,
//             Constants.Swerve.Reversed.kFrontRightDriveEncoderReversed,
//             Constants.Swerve.Reversed.kFrontRightTurningEncoderReversed,
//             Constants.Swerve.MotorPorts.kFrontRightDriveAbsoluteEncoderPort,
//             Constants.Swerve.Mechanical.kFrontRightDriveAbsoluteEncoderOffsetRad,
//             Constants.Swerve.Reversed.kFrontRightDriveAbsoluteEncoderReversed),
        
//         // Back Left
//         new SwerveModule(
//             Constants.Swerve.MotorPorts.kBackLeftDriveMotorPort,
//             Constants.Swerve.MotorPorts.kBackLeftTurningMotorPort,
//             Constants.Swerve.Reversed.kBackLeftDriveEncoderReversed,
//             Constants.Swerve.Reversed.kBackLeftTurningEncoderReversed,
//             Constants.Swerve.MotorPorts.kBackLeftDriveAbsoluteEncoderPort,
//             Constants.Swerve.Mechanical.kBackLeftDriveAbsoluteEncoderOffsetRad,
//             Constants.Swerve.Reversed.kBackLeftDriveAbsoluteEncoderReversed),
        
//         // Back Right
//         new SwerveModule(
//             Constants.Swerve.MotorPorts.kBackRightDriveMotorPort,
//             Constants.Swerve.MotorPorts.kBackRightTurningMotorPort,
//             Constants.Swerve.Reversed.kBackRightDriveEncoderReversed,
//             Constants.Swerve.Reversed.kBackRightTurningEncoderReversed,
//             Constants.Swerve.MotorPorts.kBackRightDriveAbsoluteEncoderPort,
//             Constants.Swerve.Mechanical.kBackRightDriveAbsoluteEncoderOffsetRad,
//             Constants.Swerve.Reversed.kBackRightDriveAbsoluteEncoderReversed),
        
//     };

//     // Positions stored in gyro and odometer
//     private final AHRS gyro;
//     private final SwerveDriveOdometry odometer;

//     // =)
//     public SwerveSubsystem() {
//         // Set up gyro and odometer
//         gyro = new AHRS(SPI.Port.kMXP);
//         odometer = new SwerveDriveOdometry(Constants.Swerve.Mechanical.kDriveKinematics,
//         new Rotation2d(0), 
//         new SwerveModulePosition[] {
//             modules[0].getPosition(),
//             modules[1].getPosition(),
//             modules[2].getPosition(),
//             modules[3].getPosition()
//         }, new Pose2d(5.0, 13.5, new Rotation2d()));

//         // Reset gyro
//         new Thread(() -> {
//             try {
//                 Thread.sleep(1000);
//                 // Reset position
//                 gyro.reset();
//             } catch (Exception e) {
//             }
//         }).start();
//     }

//     // Get angle robot is facing
//     public double getHeading() {
//         return Math.IEEEremainder(gyro.getAngle(), 360);
//     }

//     // Get direction robot is facing
//     public Rotation2d getRotation2d() {
//         return Rotation2d.fromDegrees(getHeading());
//     }

//     // Get position of robot
//     public Pose2d getPose() {
//         return odometer.getPoseMeters();
//     }

//     @Override
//     public void periodic() {
//         // update position
//         odometer.update(getRotation2d(), new SwerveModulePosition[] {
//             modules[0].getPosition(),
//             modules[1].getPosition(),
//             modules[2].getPosition(),
//             modules[3].getPosition()});
//         // Update distances for all modules
//         for (SwerveModule module : modules) {
//             module.updateDistance();
//         }

//         // Debug telemetry
//         SmartDashboard.putNumber("Robot Heading", getHeading());
//         SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
//     }

//     // Stop the robot
//     public void stopModules() {
//         for (int i = 0; i < modules.length; i++) {
//             modules[i].stop();
//         }
//     }

//     // DRIVE the robot
//     public void setModuleStates(SwerveModuleState[] desiredStates) {
//         // Set speed to max if go over max speed
//         SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.Mechanical.kPhysicalMaxSpeedMetersPerSecond);
//         // Move modules
//         for (int i = 0; i < modules.length; i++) {
//             modules[i].setDesiredState(desiredStates[i]);
//         }
//     }
// }
