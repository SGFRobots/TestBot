package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class drivetrain extends SubsystemBase {
    // Motors
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private TalonFX leftFollowerMotor;
    private TalonFX rightFollowerMotor;

    // Stuff abt robot?
    private static DifferentialDrive Drive;
    public double drive = 0;
    public double turn = 0;
    public boolean slow = false;
    public boolean fast = false;

    // calculation stuff
    public static final SlewRateLimiter driveFilter = new SlewRateLimiter(Constants.driveSlewRateLimit);
    public static final SlewRateLimiter turnFilter = new SlewRateLimiter(Constants.turnSlewRateLimit);

    // Encoders/gyro
    private Encoder m_leftEncoder;
    private Encoder m_rightEncoder;
    private AnalogGyro m_gyro;

    // Simulation Stuff
    private DifferentialDriveOdometry m_odometry;
    private AnalogGyroSim m_gyroSim;
    private EncoderSim m_leftEncoderSim;
    private EncoderSim m_rightEncoderSim;
    private static final Field2d m_fieldSim = new Field2d();
    private LinearSystem<N2,N2,N2> m_drivetrainSystem;
    private DifferentialDrivetrainSim m_DrivetrainSimulator;


    public drivetrain() {
        // Motors, encoders, gyro
        leftMotor = new TalonFX(Constants.MotorPorts.leftMotorPort);
        rightMotor = new TalonFX(Constants.MotorPorts.rightMotorPort);
        leftFollowerMotor = new TalonFX(Constants.MotorPorts.leftFollowerMotorPort);
        rightFollowerMotor = new TalonFX(Constants.MotorPorts.rightFollowerMotorPort);
        m_leftEncoder = new Encoder(0, 1);
        m_rightEncoder = new Encoder(2, 3);
        m_gyro = new AnalogGyro(0);

        // Simulation
        m_gyroSim = new AnalogGyroSim(m_gyro);
        m_leftEncoderSim = new EncoderSim(m_leftEncoder);
        m_rightEncoderSim = new EncoderSim(m_rightEncoder);
        m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98,0.2,1.5,0.3);
        m_DrivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,DCMotor.getCIM(2),8,Constants.Swerve.Mechanical.kTrackWidth,Constants.Swerve.Mechanical.kWheelRadius,null);

        // stuff - editors note, this really helps!

        // Get position on field
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

        // Set up motors
        leftFollowerMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
        rightFollowerMotor.setControl(new Follower(rightMotor.getDeviceID(), false));
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        Drive = new DifferentialDrive(leftMotor, rightMotor);
        Drive.setExpiration(1);
        var slot0Configs = new Slot0Configs();
        // should be in constants
        slot0Configs.kP = 0.04; // An error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.001; // A velocity of 1 rps results in 0.1 V output
        leftMotor.getConfigurator().apply(slot0Configs);
        rightMotor.getConfigurator().apply(slot0Configs);
        m_leftEncoder.setDistancePerPulse(2*Math.PI*Constants.Swerve.Mechanical.kWheelRadius/Constants.Swerve.Mechanical.kDriveEncoderResolution);
        m_rightEncoder.setDistancePerPulse(2*Math.PI*Constants.Swerve.Mechanical.kWheelRadius/Constants.Swerve.Mechanical.kDriveEncoderResolution);
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        SmartDashboard.putData("Field", m_fieldSim);
    }

    // Completely stop
    public void stop() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
    }
    
    // Drive function
    public void drive() {
        if(!RobotContainer.m_controller.getRawButton(Constants.Controllers.ButtonYPort)) {
            drive = getDrive();
            turn = getTurn();
            if(fast) {
                fastMode();
            }
            if(slow) {
                slowMode();
            }
            Drive.arcadeDrive(drive, turn);
            SmartDashboard.putNumber("Drive Value", drive);
            SmartDashboard.putNumber("Turn Value", turn);
        } else {
            stop();
        }
    }

    // Get Drive value
    public double getDrive() {
        double controllerPower = RobotContainer.m_controller.getRawAxis(Constants.Controllers.LeftYPort);
        if ((controllerPower < 0.09) && (controllerPower > -0.09)) {
            return 0;
        }
        drive = -driveFilter.calculate(controllerPower) / 2;
        return drive;
    }
    // Get Turn value
    public double getTurn() {
         double controllerPower = RobotContainer.m_controller.getRawAxis(Constants.Controllers.RightXPort);
        if ((controllerPower < 0.09) && (controllerPower > -0.09)) {
            return 0;
        }
        turn = turnFilter.calculate(controllerPower) / 2;
        // System.out.println(turn);
        return turn;
    }

    // Drive in autonomous
    public void autoDrive(double driveV, double turnV) {
        Drive.arcadeDrive(driveV, turnV);
    }

    // Speed control
    public void slowMode() {
        drive /= 2;
        turn /= 2;
    }
    public void fastMode() {
        drive *= 2;
    }

    // Update position of robot
    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(),m_leftEncoder.getDistance(),m_rightEncoder.getDistance());
    }
    // Reset position of robot
    public void resetOdometry(Pose2d pose) {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_DrivetrainSimulator.setPose(pose);
        m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
    }
    // Get position of robot - is this necessary???
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    // Update simulation
    public void simulationPeriodic() {
        m_DrivetrainSimulator.setInputs(leftMotor.get() * RobotController.getInputVoltage(), rightMotor.get() * RobotController.getInputVoltage());
        m_DrivetrainSimulator.update(0.02);
        m_leftEncoderSim.setDistance(m_DrivetrainSimulator.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_DrivetrainSimulator.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_DrivetrainSimulator.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_DrivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_DrivetrainSimulator.getHeading().getDegrees());
    }

    // Uhhh idk
    public void periodic() {
        updateOdometry();
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }
}
