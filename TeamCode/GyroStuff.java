package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private double globalAngle;
    double targetAngle = 0;
    double angleError;
    
    //timing
    private double elaspsedTime, time, timePrev;
    private double period = 1;
    
    //make current heading the zero
    public GyroStuff() { }

    public void initDrive(HardwareMecanumbot robo) {
        myRobot = robo;
        time = runtime.seconds();
    }
    public double calcPID(double target) {
        angleError = target - getGlobalAngle();
        if(Math.abs(angleError) > 5)
            return angleError*0.005;
        return 0;
    }
    public double getError(double target){
        return target - getGlobalAngle();
    }
    void resetAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    //reading angle objects z axis
    public double getAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return myRobot.angle.firstAngle;
    }
    //converting heading to global angle
    double getGlobalAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = (myRobot.angle.firstAngle+360)%360;
        return globalAngle;
    }
}
