package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class StorageSystem extends SubsystemBase {

    private final SparkMax GeneralRollers;
    private final SparkMax FeedRollers;
    private final DigitalInput irSensor1;
    private final DigitalInput irSensor2;



    public StorageSystem() {
        GeneralRollers = new SparkMax(RobotMap.STORAGR_MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);
        FeedRollers = new SparkMax(RobotMap.STORAGE_MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);
        irSensor1 = new DigitalInput(RobotMap.STORAGD_IRSENSOR1_ID);
        irSensor2 = new DigitalInput(RobotMap.STORAGE_IRSENSOR2_ID);
        SparkMaxConfig config = new SparkMaxConfig();
        GeneralRollers.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        FeedRollers.configure(config,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
    }

    public boolean atLeast1Ball(){
        return irSensor1.get() || irSensor2.get();
    }
    public boolean isFull(){
        // should be implemented with the limelight
        return false;
    }


    public void moveGeneralRollers(double speed) {
        GeneralRollers.set(speed);
    }
    public void moveFeedRollers(double speed) {
        FeedRollers.set(speed);
    }
    public void stopMotors() {
        GeneralRollers.stopMotor();
        FeedRollers.stopMotor();
    }
    public void periodic() {

    }
}
