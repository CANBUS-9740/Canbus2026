package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.StorageSim;

public class StorageSystem extends SubsystemBase {

    private final SparkMax generalRollers;
    private final SparkMax feedRollers;
    private final DigitalInput irSensor1;
    private final DigitalInput irSensor2;
    private final StorageSim sim;



    public StorageSystem() {
        generalRollers = new SparkMax(RobotMap.STORAGR_MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);
        feedRollers = new SparkMax(RobotMap.STORAGE_MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);
        irSensor1 = new DigitalInput(RobotMap.STORAGE_IRSENSOR1_ID);
        irSensor2 = new DigitalInput(RobotMap.STORAGE_IRSENSOR2_ID);
        SparkMaxConfig config = new SparkMaxConfig();
        generalRollers.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        feedRollers.configure(config,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

        if (RobotBase.isSimulation()) {
            sim = new StorageSim(generalRollers,feedRollers);
        }
        else{
            sim = null;
        }
    }

    public boolean atLeast1Ball(){
        return irSensor1.get() || irSensor2.get();
    }
    public boolean isFull(){
        // should be implemented with the limelight
        return false;
    }


    public void moveGeneralRollers(double speed) {
        generalRollers.set(speed);
    }
    public void moveFeedRollers(double speed) {
        feedRollers.set(speed);
    }
    public void stopMotors() {
        generalRollers.stopMotor();
        feedRollers.stopMotor();
    }
    public void periodic() {

    }


    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}
