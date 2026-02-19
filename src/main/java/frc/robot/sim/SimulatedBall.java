package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.dyn4j.geometry.Vector3;

public class SimulatedBall {

    private final FieldObject2d fieldObject;

    private final NetworkTableEntry positionEntry;
    private final double[] positionEntryInfo;
    private final NetworkTableEntry velocityEntry;
    private final double[] velocityEntryInfo;
    private final NetworkTableEntry accelerationEntry;
    private final double[] accelerationEntryInfo;

    private Vector3 position;
    private Vector3 velocity;
    private Vector3 acceleration;
    private boolean touchedFloor;

    public SimulatedBall(FieldObject2d fieldObject, NetworkTable table) {
        this.fieldObject = fieldObject;

        positionEntry = table.getEntry("position");
        positionEntryInfo = new double[3];
        positionEntry.setDoubleArray(positionEntryInfo);

        velocityEntry = table.getEntry("velocity");
        velocityEntryInfo = new double[3];
        velocityEntry.setDoubleArray(velocityEntryInfo);

        accelerationEntry = table.getEntry("acceleration");
        accelerationEntryInfo = new double[3];
        accelerationEntry.setDoubleArray(accelerationEntryInfo);

        position = new Vector3();
        velocity = new Vector3();
        acceleration = new Vector3();
        touchedFloor = false;

        fieldObject.setPose(new Pose2d(position.x, position.y, Rotation2d.kZero));
    }

    public Vector3 getPosition() {
        return position;
    }

    public void setPosition(Vector3 position) {
        this.position = position;
        updateEntry(positionEntry, positionEntryInfo, position);
    }

    public void setVelocity(Vector3 velocity) {
        this.velocity = velocity;
        updateEntry(velocityEntry, velocityEntryInfo, velocity);
    }

    public void setAcceleration(Vector3 acceleration) {
        this.acceleration = acceleration;
        updateEntry(accelerationEntry, accelerationEntryInfo, acceleration);
    }

    public void update(double dt) {
        // x = x0 + v0 t + 0.5 a t^2
        // v = v0 + a t
        Vector3 newVelocity = velocity.add(acceleration.product(dt));
        Vector3 newPosition = position.add(velocity.product(dt)).add(acceleration.product(dt * dt).product(0.5));

        if (!touchedFloor && newPosition.z <= 0) {
            newPosition.z = 0;
            newVelocity = new Vector3();
            acceleration = new Vector3();
            touchedFloor = true;
        }

        updateEntry(positionEntry, positionEntryInfo, newPosition);
        updateEntry(velocityEntry, velocityEntryInfo, newVelocity);
        fieldObject.setPose(new Pose2d(newPosition.x, newPosition.y, Rotation2d.kZero));

        position = newPosition;
        velocity = newVelocity;
    }

    private void updateEntry(NetworkTableEntry entry, double[] info, Vector3 data) {
        info[0] = data.x;
        info[1] = data.y;
        info[2] = data.z;
        entry.setDoubleArray(info);
    }
}
