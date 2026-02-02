package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class LimeLightFullDetection extends SubsystemBase {


    public static final double MIN_PERCENTAGE_FOR_DETECTION = 10;

    private NetworkTable table;
    private int pipeLineNum;
    public LimeLightFullDetection(String name, int pipeLineNum) {
        table  = NetworkTableInstance.getDefault().getTable(name);
        this.pipeLineNum= pipeLineNum;
        table.getEntry("pipeline").setNumber(this.pipeLineNum);
    }
    public Boolean getStorageFull( ){
        if(table.getEntry("tv").getInteger(0) == 0){
            SmartDashboard.putBoolean("isStorageFull", false);
            SmartDashboard.putString("isStorageFullReason", "No Object Detected");
            return  false;
        }
        double TargetAreaPercent = table.getEntry("ta").getDouble(0.0);

        if(TargetAreaPercent < MIN_PERCENTAGE_FOR_DETECTION){
            SmartDashboard.putBoolean("isStorageFull", false);
            SmartDashboard.putString("isStorageFullReason", "Contour isn't big enough");
            return  false;
        }

        return  true;
    }
}

