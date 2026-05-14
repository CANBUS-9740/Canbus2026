package frc.robot.commands;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeArmPositionCommand3 extends Command {
    private final IntakeArmSystem intakeArmSystem;

    private double p = 0;

    public IntakeArmPositionCommand3(IntakeArmSystem intakeArmSystem) {
        this.intakeArmSystem = intakeArmSystem;

        addRequirements(intakeArmSystem);

        SmartDashboard.putNumber("Power", 0);
    }

    @Override
    public void initialize() {
        intakeArmSystem.stop();
        p = 0;
        SmartDashboard.putNumber("Power", 0);
    }

    @Override
    public void execute() {

        double power = SmartDashboard.getNumber("Power", 0);
        if (power != this.p) {
            this.p = power;
            intakeArmSystem.set(p);
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