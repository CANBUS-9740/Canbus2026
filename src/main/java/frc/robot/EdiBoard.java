package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;

public class EdiBoard extends SubsystemBase {
    private NetworkTable table;
    ShooterSystem shooterSystem;
    IntakeCollectorSystem intakeCollectorSystem;
    ShootTurretSystem turretSystem;
    ClimbSystem climbSystem;
    GameField gameField;
    Swerve swerve;
    StorageSystem storageSystem;
    public EdiBoard(StorageSystem storageSystem, ShootTurretSystem shootTurretSystem, IntakeCollectorSystem intakeCollectorSystem, ShooterSystem shooterSystem, ClimbSystem climbSystem, GameField gameField, Swerve swerve) {
        this.turretSystem = shootTurretSystem;
        this.intakeCollectorSystem = intakeCollectorSystem;
        this.shooterSystem = shooterSystem;
        this.climbSystem = climbSystem;
        this.gameField = gameField;
        this.storageSystem = storageSystem;
        this.swerve = swerve;

        table = NetworkTableInstance.getDefault().getTable("EdiBoard");
        table.getEntry("ShooterFeeder").setBoolean(false);
        table.getEntry("ShooterJam").setBoolean(false);
        table.getEntry("ClimbLeftSwitch").setBoolean(false);
        table.getEntry("ClimbRightSwitch").setBoolean(false);

        table.getEntry("ClimbLeftPos").setDouble(0.0);
        table.getEntry("ClimbRightPos").setDouble(0.0);
        table.getEntry("TurretAngle").setDouble(0.0);
        table.getEntry("HubAngle").setDouble(0.0);

        table.getEntry("TurretLeftSwitch").setBoolean(false);
        table.getEntry("TurretRightSwitch").setBoolean(false);
        table.getEntry("TurretCenterSwitch").setBoolean(false);

        table.getEntry("ShooterAngle").setDouble(0.0);
        table.getEntry("ShooterRPM").setDouble(0.0);

        table.getEntry("ShooterSetPointSpeed").setBoolean(false);
        table.getEntry("ShooterSetPointAngle").setBoolean(false);

        table.getEntry("StorageFull").setBoolean(false);
        table.getEntry("StorageEmpty").setBoolean(false);

        table.getEntry("StorageRollerGeneral").setInteger(0);
        table.getEntry("StorageRollerFeed").setInteger(0);

        table.getEntry("IntakePos").setDouble(0.0);
        table.getEntry("IntakeArmLocation").setDouble(0.0);
    }

    @Override
    public void periodic() {
        table = NetworkTableInstance.getDefault().getTable("EdiBoard");
        table.getEntry("ShooterFeeder").setBoolean(false); // TODO: put if the ShooterFedder is running
        table.getEntry("ShooterJam").setBoolean(false);// TODO: put if the Shooter is jammed
        table.getEntry("ClimbLeftSwitch").setBoolean(climbSystem.isBottomSwitchLeftPressed());
        table.getEntry("ClimbRightSwitch").setBoolean(climbSystem.isBottomSwitchRightPressed());

        table.getEntry("ClimbLeftPos").setDouble(climbSystem.getLeftPositionMeters());
        table.getEntry("ClimbRightPos").setDouble(climbSystem.getRightPositionMeters());
        table.getEntry("TurretAngle").setDouble(turretSystem.getEncoderAngleInDegrees());
        table.getEntry("HubAngle").setDouble(gameField.getTargetAngleTurretToHub(swerve.getPose(), DriverStation.getAlliance().get()));

        table.getEntry("TurretLeftSwitch").setBoolean(turretSystem.getLimitSwitchMax());
        table.getEntry("TurretRightSwitch").setBoolean(turretSystem.getLimitSwitchMin());
        table.getEntry("TurretCenterSwitch").setBoolean(turretSystem.getLimitSwitchMin());

        table.getEntry("ShooterAngle").setDouble(shooterSystem.getPitchAngleDegrees());
        table.getEntry("ShooterRPM").setDouble(shooterSystem.getShooterVelocityRPM());

        table.getEntry("ShooterSetPointSpeed").setBoolean(false);
        table.getEntry("ShooterSetPointAngle").setBoolean(false);

        table.getEntry("StorageFull").setBoolean(storageSystem.isFull());
        table.getEntry("StorageEmpty").setBoolean(!storageSystem.atLeast1Ball());

        table.getEntry("StorageRollerGeneral").setInteger(1);
        table.getEntry("StorageRollerFeed").setInteger(1);

        table.getEntry("IntakePos").setDouble(0.0);
        table.getEntry("IntakeArmLocation").setDouble(0.0);
    }

    public boolean getShooterBtnPressed(){
        return table.getEntry("ShootBTN").getBoolean(false);
    }


    public boolean getIntakeBtnPressed(){
        return table.getEntry("IntakeBTN").getBoolean(false);
    }


    public boolean getClibUpBtnPressed(){
        return table.getEntry("ClimbUpBTN").getBoolean(false);
    }

    public boolean getClimbDownBtnPressed(){
        return table.getEntry("ClimbUpBTN").getBoolean(false);
    }
}
