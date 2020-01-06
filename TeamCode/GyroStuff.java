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
    double angleError;
    
    //make current heading the zero
    public GyroStuff() { }

    public void initDrive(HardwareMecanumbot robo) {
        myRobot = robo;
    }
    
    public double calcPID(double target) {
        angleError = target - getAngle();
        if(Math.abs(angleError) > 0.5)
            return Range.clip(angleError/100.0, -0.6, 0.6);
        return 0;
    }
    void resetAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    //reading angle objects z axis
    public double getAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return myRobot.angle.firstAngle;
    }
}
