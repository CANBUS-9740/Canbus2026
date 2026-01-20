package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeArmPositionCommand extends Command {
    IntakeArmSystem intakeArmSystem;
    double targetAngle;

    public IntakeArmPositionCommand(IntakeArmSystem intakeArmSystem, double targetAngle) {
        this.intakeArmSystem= intakeArmSystem;
        this.targetAngle = targetAngle;
        addRequirements();
    }

    public void initialize() {

    }

    public void execute() {
        intakeArmSystem.movePosePID(targetAngle);
    }


    public void end(boolean interrupted) {
        intakeArmSystem.stop();
    }


    public boolean isFinished() {
        return false;
    }



}


