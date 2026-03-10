package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeArmPositionCommand extends Command {
    private final IntakeArmSystem intakeArmSystem;
    private final double targetPositionDegrees;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    public IntakeArmPositionCommand(IntakeArmSystem intakeArmSystem, double targetPositionDegrees) {
        this.intakeArmSystem= intakeArmSystem;
        this.targetPositionDegrees = targetPositionDegrees;
        addRequirements(intakeArmSystem);
    }
    @Override
    public void initialize() {
        intakeArmSystem.setTargetPosition(targetPositionDegrees);
    }
    @Override
    public void execute() {
        motionProfile = new TrapezoidProfile(RobotMap.INTAKE_ARM_MOTION_PROFILE_CONSTRAINTS);
        motionProfileGoal = new TrapezoidProfile.State(targetPositionDegrees, 0);
        motionProfileSetPoint = new TrapezoidProfile.State(intakeArmSystem.getPositionDegrees(), 0);
        motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);

        intakeArmSystem.setTargetPosition(motionProfileSetPoint.position);
    }

    @Override
    public void end(boolean interrupted) {
        intakeArmSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return intakeArmSystem.IsArmInPositionAndSteady(targetPositionDegrees);
    }



}


