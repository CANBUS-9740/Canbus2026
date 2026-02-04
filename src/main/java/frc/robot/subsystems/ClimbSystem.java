package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.ClimbSim;

public class ClimbSystem extends SubsystemBase {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final DigitalInput bottomSwitchLeft;
    private final DigitalInput bottomSwitchRight;
    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;

    private final ClimbSim sim;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d leftLigament;
    private final MechanismLigament2d rightLigament;


    public ClimbSystem() {
        leftMotor = new SparkMax(RobotMap.CLIMB_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(RobotMap.CLIMB_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoderLeft = leftMotor.getEncoder();
        encoderRight = rightMotor.getEncoder();

        bottomSwitchLeft = new DigitalInput(RobotMap.CLIMB_BOTTOM_SWITCH_LEFT_SENSOR_ID);
        bottomSwitchRight = new DigitalInput(RobotMap.CLIMB_BOTTOM_SWITCH_RIGHT_SENSOR_ID);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0.5,0,0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        config = new SparkMaxConfig();
        config.closedLoop.pid(0.5,0,0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (RobotBase.isSimulation()) {
            sim = new ClimbSim(leftMotor,rightMotor);
        } else {
            sim = null;
        }

        mechanism = new Mechanism2d(5,5);
        MechanismRoot2d leftRoot = mechanism.getRoot("leftArm",2,2);
        MechanismRoot2d rightRoot = mechanism.getRoot("rightArm",3,2);
        leftLigament = leftRoot.append(new MechanismLigament2d("leftArm",0,90,5,new Color8Bit(Color.kRed)));
        rightLigament = rightRoot.append(new MechanismLigament2d("rightArm",0,90,5,new Color8Bit(Color.kRed)));
        SmartDashboard.putData("climb", mechanism);
    }

    public double getLeftPositionMeters() {
        return encoderLeft.getPosition() * RobotMap.CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS;
    }

    public double getRightPositionMeters() {
        return encoderRight.getPosition() * RobotMap.CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS;
    }

    public void setMotors(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void stopLeft() {
        leftMotor.stopMotor();
    }

    public void stopRight() {
        rightMotor.stopMotor();
    }

    public void setTargetPosition(double positionMeters) {
        leftMotor.getClosedLoopController().setSetpoint(positionMeters/RobotMap.CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS, SparkBase.ControlType.kPosition);
        rightMotor.getClosedLoopController().setSetpoint(positionMeters/RobotMap.CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS, SparkBase.ControlType.kPosition);
    }

    public boolean isAtLeftTarget(double targetPosition) {
        return MathUtil.isNear(targetPosition,getLeftPositionMeters(), RobotMap.CLIMB_ARMS_TARGET_TOLERANCE) && Math.abs(encoderLeft.getVelocity()) < RobotMap.CLIMB_ARMS_TARGET_RPM_TOLERANCE;
    }

    public boolean isAtRightTarget(double targetPosition) {
        return MathUtil.isNear(targetPosition,getRightPositionMeters(), RobotMap.CLIMB_ARMS_TARGET_TOLERANCE) && Math.abs(encoderRight.getVelocity()) < RobotMap.CLIMB_ARMS_TARGET_RPM_TOLERANCE;
    }

    public boolean isBottomSwitchLeftPressed() {
        return bottomSwitchLeft.get();
    }

    public boolean isBottomSwitchRightPressed() {
        return bottomSwitchRight.get();
    }

    @Override
    public void periodic() {
        double leftLength = getLeftPositionMeters();
        SmartDashboard.putNumber("ClimbArmLeftPosition", leftLength);
        leftLigament.setLength(leftLength + 0.1);

        double rightLength = getRightPositionMeters();
        SmartDashboard.putNumber("ClimbArmRightPosition", rightLength);
        rightLigament.setLength(rightLength + 0.1);

        SmartDashboard.putBoolean("climbLeftSwitch", isBottomSwitchLeftPressed());
        SmartDashboard.putBoolean("climbRightSwitch", isBottomSwitchRightPressed());
    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}
