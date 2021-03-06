package frc.swervemath;

import javax.swing.GrayFilter;

import frc.swervemath.Hardware.Gyro;
import frc.swervemath.math.Vector2;
import frc.swervemath.math.Vector3;

public class DriveTrain {
    public final Wheel[] wheels;
    public final Gyro gyro;

    public DriveTrain(Gyro gyro, Wheel... wheels){
        this.wheels = wheels;
        this.gyro = gyro;
    }

    public void robotOrientedDrive(Vector2 Heading, float rot){
        //adds deadzones to input
        if(Heading.magnitude() > -0.1f && Heading.magnitude() < 0.1f)
            Heading = Vector2.zero();
        if(rot < 0.1f && rot > -0.1f)
            rot = 0;

        if(rot == 0){
            //Without rotation, there isn't a point to be tangent to, just a direction to move
            float speed = Heading.magnitude();

            //maxes the speed out at 100%
            speed = speed > 1 ? 1 : speed;
            if(Heading.magnitude() != 0){
                for(int i = 0; i < wheels.length; i++){
                    wheels[i].setAngleAndSpeed((float)Math.atan2(Heading.x, -Heading.y), speed);
                }
            } else {
                //if the Heading vector is zero, then Math.atan2 will raise an exception
                for(int i = 0; i < wheels.length; i++){
                    wheels[i].setAngleAndSpeed(0.0f, 0.0f);
                }
            }
        } else {
            //balances rotation with strafing and makes the point on a line perpendicular to the heading to control the direction of movement
            Vector2 pointOfRot = Vector2.multiply(new Vector2(Heading.y, -Heading.x).normalized(), Heading.magnitude() / rot);

            float max = 0;

            float[] angles = new float[wheels.length];
            float[] mags = new float[wheels.length];

            for(int i = 0; i < wheels.length; i ++){
                Wheel wheel = wheels[i];

                //Rotates the delta 90 degrees dependent on the intended direction to spin
                if(rot > 0)
                    angles[i] = (float)Math.atan2(-pointOfRot.x + wheel.pos.x, -(-pointOfRot.y + wheel.pos.y));
                else
                    angles[i] = (float)Math.atan2(-(-pointOfRot.x + wheel.pos.x), -pointOfRot.y + wheel.pos.y);
                mags[i] = Vector2.subtract(wheel.pos, pointOfRot).magnitude();

                //finds the maximum distance from the point of rot to a wheel
                max = (mags[i] > max) ? mags[i] : max;
            }
            float speed = new Vector3(Heading.x, Heading.y, rot).magnitude();

            //maxes the speed out at 100%
            speed = speed > 1 ? 1 : speed;

            for(int i = 0; i < wheels.length; i ++)
                wheels[i].setAngleAndSpeed(angles[i], (mags[i] / max) * speed);
        }
    }
    public void fieldOrientedDrive(Vector2 Heading, float rot){
        //rotates the heading input by the negative rotation of the bot to ensure that it stays field-oriented
        float cos = (float)Math.cos(-gyro.angleInRads());
        float sin = (float)Math.sin(-gyro.angleInRads());

        System.out.println("Angle of Bot: " + gyro.angleInRads() * 180/Math.PI);

        robotOrientedDrive(new Vector2(Heading.x * cos - Heading.y * sin, Heading.y * cos + Heading.x * sin), rot);
    }
    public Gyro getGyro(){
        return gyro;
    }
}
