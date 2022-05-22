package frc.swervemath.Hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class WCPSwerve implements WheelHardware {
    private final TalonFX angle;
    private final TalonFX speed;

    int id;

    private final CANCoder encoder;
    private final float offset;

    private float setpoint;

    private final float angleToWheelRatio = 72/7;

    public WCPSwerve(int angleID, int speedID, int encoderID, int id){
        this.id = id;
        angle = new TalonFX(angleID);
        speed = new TalonFX(speedID);

        encoder = new CANCoder(encoderID);
        offset = (float)encoder.getAbsolutePosition();

        setpoint = 0;
    }
    public float getEncoderOut(){
        return ((((float)encoder.getAbsolutePosition()-offset)%360)+360)%360;
    }
    float angleFactor(){
        float delta = getEncoderOut()-setpoint;
        delta *= Math.PI/180;
        return (float)-Math.cos(delta);
    }
    public float PIDEncOut(){
        return getEncoderOut() % 180;
    }
    public float getError(){
        setpoint %= 180;
    
        float loopDownError = -((180 - setpoint) + PIDEncOut());
        float loopUpError = ((180 - PIDEncOut()) + setpoint);
    
        float loopError = Math.abs(loopUpError) > Math.abs(loopDownError) ? loopDownError : loopUpError;
    
        float nonLoopError = (setpoint%180) - PIDEncOut();
        
        float error = (Math.abs(nonLoopError) > Math.abs(loopError)) ? loopError : nonLoopError;
    
        return (error);
      }
    public void setSpeed(float speed){
        this.speed.set(ControlMode.PercentOutput, speed * angleFactor());
    }
    
    public void setSetpoint(float setpoint){
        setpoint *= -180/(float)Math.PI;
        this.setpoint = setpoint;

        angle.set(ControlMode.Position, angle.getSelectedSensorPosition() - (getError() * angleToWheelRatio * 360));
    }
}
