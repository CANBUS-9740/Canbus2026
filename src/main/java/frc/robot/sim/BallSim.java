package frc.robot.sim;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import org.dyn4j.geometry.Vector3;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class BallSim {

    private final Field2d field;
    private final NetworkTable baseTable;
    private final List<SimulatedBall> balls;
    private int nextBallIndex;

    public BallSim(Field2d field) {
        this.field = field;

        baseTable = NetworkTableInstance.getDefault().getTable("balls");
        balls = new ArrayList<>();
        nextBallIndex = 0;
    }

    public void launchBall(Translation3d launchPosition, double shooterDirectionDegrees, double shooterRpm, double firingAngleDegrees) {
        double firingVelocity = shooterRpm * 2 * Math.PI * RobotMap.SHOOTER_WHEEL_RADIUS_METERS / 60;
        double shooterDirection = Math.toRadians(shooterDirectionDegrees);
        double firingAngle = Math.toRadians(firingAngleDegrees);

        double velX = firingVelocity * Math.sin(firingAngle) * Math.cos(shooterDirection);
        double velY = firingVelocity * Math.sin(firingAngle) * Math.sin(shooterDirection);
        double velZ = firingVelocity * Math.cos(firingAngle);

        Vector3 position = new Vector3(launchPosition.getX(), launchPosition.getY(), launchPosition.getZ());
        Vector3 velocity = new Vector3(velX, velY, velZ);
        Vector3 acceleration = new Vector3(0, 0, -9.8);

        int ballId = nextBallIndex++;
        String ballName = String.format(Locale.ENGLISH, "Ball%d", ballId);
        FieldObject2d fieldObject = field.getObject(ballName);
        NetworkTable table = baseTable.getSubTable(ballName);
        SimulatedBall ball = new SimulatedBall(fieldObject, table);

        ball.setPosition(position);
        ball.setVelocity(velocity);
        ball.setAcceleration(acceleration);

        balls.add(ball);
    }

    public void update() {
        for (SimulatedBall ball : balls) {
            ball.update(0.02);
        }
    }
}
