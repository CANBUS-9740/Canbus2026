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
        this.intakeArmSystem = intakeArmSystem;
        this.targetPositionDegrees = targetPositionDegrees;

        addRequirements(intakeArmSystem);
    }

    @Override
    public void initialize() {
        System.out.println("123");

        motionProfile = new TrapezoidProfile(RobotMap.INTAKE_ARM_MOTION_PROFILE_CONSTRAINTS);
        motionProfileGoal = new TrapezoidProfile.State(targetPositionDegrees, 0);
        motionProfileSetPoint = new TrapezoidProfile.State(intakeArmSystem.getPositionDegrees(), 0);
    }

    @Override
    public void execute() {
        motionProfileSetPoint = motionProfile.calculate(
                0.02,
                motionProfileSetPoint,
                motionProfileGoal
        );

        intakeArmSystem.setTargetPosition(motionProfileSetPoint.position);
        System.out.println("Profile position: " + motionProfileSetPoint.position);
        System.out.println("Arm position: " + intakeArmSystem.getPositionDegrees());
        System.out.println("Goal position: " + motionProfileGoal.position);
    }

    @Override
    public void end(boolean interrupted) {
        intakeArmSystem.stop();
        System.out.println("nn");
        System.out.println( " pose " + intakeArmSystem.getPositionDegrees() );
    }

    @Override
    public boolean isFinished() {
        return intakeArmSystem.IsArmInPositionAndSteady(targetPositionDegrees);
    }
}