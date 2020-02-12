package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class GyroStuff {
    //define class members
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareMecanumbot myRobot;
    
    //angle variables
    double angleError;
    
    //make current heading the zero
    public GyroStuff() { }

    public void initDrive(HardwareMecanumbot robo) {
        myRobot = robo;
        resetAngle();
    }
    
    public double calcPID(double target) {
        angleError = target - getAngle();
        
        //do an angle wrap so -180 < angleError < 180
        if(angleError < -180)
            angleError += 360;
        if(angleError > 180)
            angleError -= 360;
        
        //if at target angle stop
        if(Math.abs(angleError) < 0.5)
            return 0;
            
        //make sure that for really close values the power is not negligeable
        if(angleError < 3.0 && angleError >= 0)
            return 0.05;
        if(angleError > -3.0 && angleError < 0)
            return -0.05;
        
        //send higher power values for higher values of angleError
        return Range.clip(angleError/30.0, -1, 1);
    }
    void resetAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public double getError() {
        return angleError;
    }
    //reading angle objects z axis
    public double getAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return myRobot.angle.firstAngle;
    }
}