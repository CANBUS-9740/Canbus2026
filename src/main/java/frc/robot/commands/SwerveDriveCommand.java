package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;

public class SwerveDriveCommand extends Command {
    private final Swerve swerveSub;
    private final CommandXboxController xboxController;
    private final boolean fieldDrive;
    private double allianceSwapper = 1;

    public SwerveDriveCommand(Swerve swerveSub, CommandXboxController xboxController, boolean fieldDrive) {
        this.swerveSub = swerveSub;
        this.xboxController = xboxController;
        this.fieldDrive = fieldDrive;

        addRequirements(swerveSub);
    }

    @Override
    public void initialize() {
        swerveSub.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public void execute() {
        double x = allianceSwapper * -MathUtil.applyDeadband(xboxController.getLeftY(), 0.1) * RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS * 2;
        double y = allianceSwapper * -MathUtil.applyDeadband(xboxController.getLeftX(), 0.1) * RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS * 2;
        double rotation = -MathUtil.applyDeadband(xboxController.getRightX(), 0.1) * 2;

        if (fieldDrive) {
            swerveSub.driveFieldRelative(x, y, rotation);
        } else {
            swerveSub.drive(new ChassisSpeeds(x, y, rotation));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.stop();
    }

    public void setAllianceSwapper() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            allianceSwapper = -1;
        } else {
            allianceSwapper = 1;
        }
    }
}
