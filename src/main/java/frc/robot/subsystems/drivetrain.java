package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.DrivewithJoysticks;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class drivetrain extends SubsystemBase {
    // Motors
    TalonFX leftMotor;
    TalonFX rightMotor;
    TalonFX leftFollowerMotor;
    TalonFX rightFollowerMotor;

    // Stuff
    DifferentialDrive Drive;
    double setPosition = 0;
    double drive;
    double turn;
    private static final double kTrackWidth = 0.381 * 2;
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = -4096;

    PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    public static SlewRateLimiter driveFilter = new SlewRateLimiter(Constants.driveSlewRateLimit);
    public static SlewRateLimiter turnFilter = new SlewRateLimiter(Constants.turnSlewRateLimit);

    // Simulation stuff
    Encoder m_leftEncoder;
    Encoder m_rightEncoder;
    private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);
    AnalogGyro m_gyro;

    DifferentialDriveKinematics m_kinematics;
    DifferentialDriveOdometry m_odometry;

    private final SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(1, 3);

    AnalogGyroSim m_gyroSim;
    EncoderSim m_leftEncoderSim;
    EncoderSim m_rightEncoderSim;
    Field2d m_fieldSim = new Field2d();
    
    LinearSystem<N2,N2,N2> m_drivetrainSystem;
    DifferentialDrivetrainSim m_DrivetrainSimulator;


    public drivetrain() {
        leftMotor = new TalonFX(Constants.leftMotorPort);
        rightMotor = new TalonFX(Constants.rightMotorPort);
        leftFollowerMotor = new TalonFX(Constants.leftFollowerMotorPort);
        rightFollowerMotor = new TalonFX(Constants.rightFollowerMotorPort);

        m_leftEncoder = new Encoder(0, 1);
        m_rightEncoder = new Encoder(2, 3);
        m_gyro = new AnalogGyro(0);

        // Simulation
        m_gyroSim = new AnalogGyroSim(m_gyro);
        m_leftEncoderSim = new EncoderSim(m_leftEncoder);
        m_rightEncoderSim = new EncoderSim(m_rightEncoder);
        m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98,0.2,1.5,0.3);
        m_DrivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,DCMotor.getCIM(2),8,kTrackWidth,kWheelRadius,null);

        // stuff - editors note, this really helps!
        m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

        // Get position on field
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

        leftFollowerMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
        rightFollowerMotor.setControl(new Follower(rightMotor.getDeviceID(), false));
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        Drive = new DifferentialDrive(leftMotor, rightMotor);
        Drive.setExpiration(1);
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.04; // An error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.001; // A velocity of 1 rps results in 0.1 V output
        leftMotor.getConfigurator().apply(slot0Configs);
        rightMotor.getConfigurator().apply(slot0Configs);

        m_leftEncoder.setDistancePerPulse(2*Math.PI*kWheelRadius/kEncoderResolution);
        m_rightEncoder.setDistancePerPulse(2*Math.PI*kWheelRadius/kEncoderResolution);
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        SmartDashboard.putData("Field", m_fieldSim);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speed) {
        var leftFeedForward = m_Feedforward.calculate(speed.leftMetersPerSecond);
        var rightFeedForward = m_Feedforward.calculate(speed.rightMetersPerSecond);
        double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speed.leftMetersPerSecond);
        double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speed.rightMetersPerSecond);

        leftMotor.setVoltage(leftOutput + leftFeedForward);
        rightMotor.setVoltage(rightOutput + rightFeedForward);
    }
    public void drive(double drive, double turn) {
        setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(drive, 0, turn)));
    }

    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(),m_leftEncoder.getDistance(),m_rightEncoder.getDistance());
    }
    public void resetOdometry(Pose2d pose) {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_DrivetrainSimulator.setPose(pose);
        m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
    }
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    public void simulationPeriodic() {
        m_DrivetrainSimulator.setInputs(leftMotor.get() * RobotController.getInputVoltage(), rightMotor.get() * RobotController.getInputVoltage());
        m_DrivetrainSimulator.update(0.02);
        m_leftEncoderSim.setDistance(m_DrivetrainSimulator.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_DrivetrainSimulator.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_DrivetrainSimulator.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_DrivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_DrivetrainSimulator.getHeading().getDegrees());
    }

    public void periodic() {
        updateOdometry();
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }
    
    public void arcadeDrive() {
        // if(!RobotContainer.m_controller.getRawButton(Constants.buttonHPort)) {
        //     drive = driveFilter.calculate(Util.inputCurve(RobotContainer.m_controller.getRawAxis(Constants.driveAxis), 1));
        //     turn = driveFilter.calculate(Util.inputCurve(RobotContainer.m_controller.getRawAxis(Constants.turnAxis), 1));
        // } else {
        //     drive = 0;
        //     turn = 0;
        // }
        drive = driveFilter.calculate(Util.inputCurve(RobotContainer.m_controller.getLeftY(),1));
        turn = driveFilter.calculate(Util.inputCurve(RobotContainer.m_controller.getRightX(), 1));
        drive(drive, turn);
        SmartDashboard.putNumber("Drive", drive);
        SmartDashboard.putNumber("Turn", turn);
    }
}
