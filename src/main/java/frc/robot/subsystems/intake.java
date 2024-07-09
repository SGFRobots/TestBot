package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class intake extends SubsystemBase {
    // Intake motors
    public final CANSparkMax m_bottomRoller;
    public final CANSparkMax m_topRoller;
    public final CANSparkMax m_intakeBelt;
    public final double power = 0.5;

    public intake() {
        // Motors
        m_bottomRoller = new CANSparkMax(Constants.bottomIntakeRollerMotor, MotorType.kBrushless);
        m_topRoller = new CANSparkMax(Constants.topIntakeRollerMotor, MotorType.kBrushless);
        m_intakeBelt = new CANSparkMax(Constants.intakeBeltMotor, MotorType.kBrushless);

        // Set direction
        m_bottomRoller.setInverted(false);
        m_topRoller.setInverted(false);
    }

    @Override
    public void periodic() {
        // Update power
        SmartDashboard.putNumber("Intake Value", m_bottomRoller.get());
    }

    // TeleOp intake
    public void scoop() {
        m_bottomRoller.set(power);
        m_topRoller.set(power);
        m_intakeBelt.set(power);
    }

    // Stop completely
    public void stop() {
        m_bottomRoller.set(0);
        m_topRoller.set(0);
        m_intakeBelt.set(0);
    }

    // Autonomous intake
    public void autoScoop(double power) {
        m_bottomRoller.set(power);
        m_topRoller.set(power);
        m_intakeBelt.set(power);
    }
}
