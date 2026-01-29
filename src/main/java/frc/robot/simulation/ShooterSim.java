package frc.robot.simulation;


import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class ShooterSim extends SubsystemBase {
    private final ShooterSystem shooterSystem = new ShooterSystem();
    private final SparkMaxSim shooterLeadSim = new SparkMaxSim(shooterSystem.getShooterMotorMain(), DCMotor.getNEO(1));
    private final SparkMaxSim shooterFollowSim = new SparkMaxSim(shooterSystem.getShooterMotorFollow(), DCMotor.getNEO(1));
    private final SparkMaxSim shooterPitchMotorSim = new SparkMaxSim(shooterSystem.getPitchMotor(), DCMotor.getNEO(1));
    private final SparkMaxSim shooterFeederMotorSim = new SparkMaxSim(shooterSystem.getFeederMotor(), DCMotor.getNEO(1));

    private final XboxController controller = new XboxController(0);
    private final XboxController controllerSim = controller;

    private final FlywheelSim s_FlywheelSim;

    public ShooterSim() {
        s_FlywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1)
                , RobotMap.MOMENT_OF_INERTIA
                , RobotMap.SHOOTER_PITCH_GEAR_RATIO
        ), DCMotor.getNEO(1), 0);

        SparkMaxConfig configFollowSim = new SparkMaxConfig();


    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("angle", shooterPitchMotorSim.getPosition() * 90);
    }


    @Override
    public void simulationPeriodic() {
        if (controller.getAButton()) {
            shooterLeadSim.setVelocity(1500);
            shooterFollowSim.setVelocity(-1500);
        }
        if (controller.getBButton()) {
            shooterLeadSim.setVelocity(1000);
            shooterFollowSim.setVelocity(-1000);
        }

    }
}
