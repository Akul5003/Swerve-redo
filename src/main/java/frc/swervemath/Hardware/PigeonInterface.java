package frc.swervemath.Hardware;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;

public class PigeonInterface implements Gyro {
    private final Pigeon2 pigeon;
    private float calibrationAngle;

    public PigeonInterface(int pigID){
        pigeon = new Pigeon2(pigID, "Default Name");
    }
    public void zero(){
        calibrationAngle = -(float)pigeon.getYaw();
    }
    public float angleInRads(){
        return ((float)(-(pigeon.getYaw())) - calibrationAngle) * (float)Math.PI/180;
    }
}
