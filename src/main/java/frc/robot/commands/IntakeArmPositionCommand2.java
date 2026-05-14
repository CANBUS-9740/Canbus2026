package frc.robot.commands;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeArmPositionCommand2 extends Command {
    private final IntakeArmSystem intakeArmSystem;

    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double sp = 0;
    private double ks = 0;

    public IntakeArmPositionCommand2(IntakeArmSystem intakeArmSystem) {
        this.intakeArmSystem = intakeArmSystem;

        addRequirements(intakeArmSystem);

        intakeArmSystem.config.closedLoop.pid(0, 0, 0).feedForward.kS(0);
        intakeArmSystem.motor.configure(intakeArmSystem.config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
        SmartDashboard.putNumber("kS", 0);
        SmartDashboard.putNumber("SetPoint", 0);
        SmartDashboard.putNumber("ProcessVariable", 0);
    }

    @Override
    public void initialize() {
        intakeArmSystem.stop();
        sp = 0;
        SmartDashboard.putNumber("SetPoint", 0);
    }

    @Override
    public void execute() {
        boolean configChanged = false;

        double kp = SmartDashboard.getNumber("kP", 0);
        if (kp != this.kp) {
            this.kp = kp;
            intakeArmSystem.config.closedLoop.p(kp);
            configChanged = true;
        }
        double ki = SmartDashboard.getNumber("kI", 0);
        if (ki != this.ki) {
            this.ki = ki;
            intakeArmSystem.config.closedLoop.i(ki);
            configChanged = true;
        }
        double kd = SmartDashboard.getNumber("kD", 0);
        if (kd != this.kd) {
            this.kd = kd;
            intakeArmSystem.config.closedLoop.d(kd);
            configChanged = true;
        }
        double ks = SmartDashboard.getNumber("kS", 0);
        if (ks != this.ks) {
            this.ks = ks;
            intakeArmSystem.config.closedLoop.feedForward.kS(ks);
            configChanged = true;
        }

        if (configChanged) {
            intakeArmSystem.motor.configure(intakeArmSystem.config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        double setPoint = SmartDashboard.getNumber("SetPoint", 0);
        if (setPoint != sp) {
            sp = setPoint;
            intakeArmSystem.setTargetPosition(sp);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeArmSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}