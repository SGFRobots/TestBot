package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    // motors and encoders
    public final CANSparkMax driveMotor;
    public final CANSparkMax turnMotor;
    public final RelativeEncoder driveEncoder;
    public final RelativeEncoder turnEncoder;

    public final PIDController turningPIDController;

    // aBSOLUTE ENCODER
    public final AnalogInput absoluteEncoder;
    public final boolean absoluteEncoderReversed;
    public final double absoluteEncoderOffset;

    public SwerveModule(
        int drivePort, int turnPort, boolean driveReversed, boolean turnReversed, int absoluteEncoderPort, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

            // Motors
            driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
            driveMotor.setInverted(driveReversed);
            turnMotor.setInverted(turnReversed);

            // Encoders
            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getEncoder();

            // Conversions
            driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter);
            driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec);
            turnEncoder.setPositionConversionFactor(Constants.kTurningEncoderRot2Rad);
            turnEncoder.setVelocityConversionFactor(Constants.kTurningEncoderRPM2RadPerSec);

            // Absolute Encoder
            absoluteEncoder = new AnalogInput(absoluteEncoderPort);
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            this.absoluteEncoderOffset = absoluteEncoderOffset;

            //PID Controller
            turningPIDController = new PIDController(0.5, 0, 0);
            turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

            resetEncoder();

    }

    // we probably wont need these next 4 =)
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }

    // Set all position to 0
    public void resetEncoder() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    // get current angle in radians
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffset;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    // Return the object that has all data of the position of the robot
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    // Move
    public void setDesiredState(SwerveModuleState state) {
        // Don't move back to 0 after moving
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        //Move
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.maxSpeed);
        turnMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
