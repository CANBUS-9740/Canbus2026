package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeArmDropCommand extends Command {

    private static final double TARGET_DROP_POSITION = 70;

    private final IntakeArmSystem system;
    private boolean isHolding;

    public IntakeArmDropCommand(IntakeArmSystem system) {
        this.system = system;
        addRequirements(system);
    }

    @Override
    public void initialize() {
        system.setTargetPosition(TARGET_DROP_POSITION);
        isHolding = true;
    }

    @Override
    public void execute() {
        if (isHolding && system.IsArmInPositionAndSteady(TARGET_DROP_POSITION)) {
            isHolding = false;
            system.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        system.stop();
    }

    @Override
    public boolean isFinished() {
        return system.IsArmInPositionAndSteady(0);
    }
}
