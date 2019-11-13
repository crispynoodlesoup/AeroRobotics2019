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

public class GyroMath {
    //define class members
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareTestbot myRobot;
    
    //angle variables
    private double globalAngle;
    double target_Angle = 0;
    private double prev_angle_error = 0;
    double angle_error;
    
    //distance variables - LATER
    double distance = 0;
    double prev_dist_error, dist_error;
    double target_Distance = 1;
    
    //timing
    private double elaspsedTime, time, timePrev;
    private double period = 1;
    
    //variables for the PID systems
    double kP = 0.005;
    double kI = 0.001;
    double kD = 0.05;
    //actual PID outputs
    double PID_p, PID_i = 0, PID_d = 0, PID_total;
    //make current heading the zero
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
