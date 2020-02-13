package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="BlueColorSensorAuto", group="generated")
public class ColorAuto_Blue extends LinearOpMode {
        //width = 18.0; //inches
        //cpr = 383.6; //counts per rotation
        //gearratio = 13.7;
        //diameter = 3.937;
        //cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, cpr * gear ratio / (2 * pi * diameter)
        //bias = 0.205;
        int conversion = 88;
        
        // meccyBias = 0.21;//change to adjust only strafing movement
        int straifConv = 89;
        
        int colorOffset  = 0;
        int colorOffset2 = 0;
        
        int r;
        
        Boolean exit = false;
        double redSide = -1.0;
        ElapsedTime runtime = new ElapsedTime();
        GyroStuff gyro = new GyroStuff();
        HardwareMecanumbot robot = new HardwareMecanumbot();
        @Override
        public void runOpMode(){
                robot.initDrive(this);
                gyro.initDrive(robot);
                
                //robot.liftPos(17000);
                //sleep(500);
                robot.servoGrab1.setPosition(1);
                robot.servoGrab2.setPosition(0.6);
                robot.servoArm.setPosition(0.4);
                //
                waitForStart();
                //
                strafeToPosition(-32.5, 0.95);
                //
                scanColor();
                //
                moveToPosition(-5.5*redSide, 0.7);
                //
                strafeToPosition(-2.0, 0.5);
                //
                robot.servoArm.setPosition(0.95);
                //
                sleep(600);
                //
                strafeToPosition(10, 0.9);
                //
                moveToPosition(50.5*redSide + colorOffset*redSide, 0.95);
                //
                robot.servoArm.setPosition(0.3);
                //
                sleep(600);
                //
                moveToPosition(-42.0*redSide + colorOffset2*redSide, 0.95);
                //
                strafeToPosition(-13.5, 0.8);
                //
                robot.servoArm.setPosition(0.95);
                //
                sleep(600);
                //
                strafeToPosition(12.0, 0.8);
                //
                moveToPosition(44.0*redSide - colorOffset2*redSide, 0.95);
                //uwu
                robot.servoArm.setPosition(0.3);
                //
                sleep(600);
                //
                moveToPosition(-16.0*redSide, 0.9);
                //
                strafeToPosition(8.0, 1.0);
                }
        public void moveToPosition(double inches, double speed){
                int move  = (int)(Math.round(inches * conversion)) + 30;
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                int decelInches = (int)Math.abs(Math.round(move * 18/26));
                decelInches = (int)Range.clip(decelInches, 0, 1800);
                double speedTemp = speed;
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + move);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() + move);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                if(inches < 0)
                        r = -1;
                else
                        r = 1;
                //
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 800) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                speed = Range.clip(speed, 0.1, 1.0);
                driveStraight(0,speed,r);
                //
                //while (((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 || end*r - robot.leftFront.getCurrentPosition()*r > 90) && !isStopRequested()){
                //while ((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 && !isStopRequested()){
                while (end*r - robot.leftFront.getCurrentPosition()*r > 30 && !isStopRequested()){
                        speed = speedTemp;
                        if(Math.abs(robot.leftFront.getCurrentPosition()*r - start*r) < 800) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                        if(Math.abs(end*r - robot.leftFront.getCurrentPosition()*r) < decelInches) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/1800.0;
                        speed = Range.clip(speed, 0.1, 1.0);
                        driveStraight(0,speed,r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("leftF", "%.3f", robot.leftF);
                        telemetry.addData("rightF", "%.3f", robot.rightF);
                        telemetry.addData("leftR", "%.3f", robot.leftR);
                        telemetry.addData("rightR", "%.3f", robot.rightR);
                        telemetry.addData("end", "%d", (end*r - robot.leftFront.getCurrentPosition()*r));
                        telemetry.update();
                }
                turnToTarget(0);
        }
        public void strafeToPosition(double inches, double speed){
                int move  = (int)(Math.round(inches * conversion)) + 30;
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                int decelInches = (int)Math.abs(Math.round(move * 18/26));
                decelInches = (int)Range.clip(decelInches, 0, 1800);
                double speedTemp = speed;
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() - move);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() - move);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                if(inches < 0)
                        r = -1;
                else
                        r = 1;
                //
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 800) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                speed = Range.clip(speed, 0.1, 1.0);
                driveStraight(0,speed,r);
                //
                //while (((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 || end*r - robot.leftFront.getCurrentPosition()*r > 90) && !isStopRequested()){
                //while ((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 && !isStopRequested()){
                while (end*r - robot.leftFront.getCurrentPosition()*r > 30 && !isStopRequested()){
                        speed = speedTemp;
                        if(Math.abs(robot.leftFront.getCurrentPosition()*r - start*r) < 800) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                        if(Math.abs(end*r - robot.leftFront.getCurrentPosition()*r) < decelInches) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/1800.0;
                        speed = Range.clip(speed, 0.1, 1.0);
                        driveStraight(0,speed,r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("leftF", "%.3f", robot.leftF);
                        telemetry.addData("rightF", "%.3f", robot.rightF);
                        telemetry.addData("leftR", "%.3f", robot.leftR);
                        telemetry.addData("rightR", "%.3f", robot.rightR);
                        telemetry.addData("end", "%d", (end*r - robot.leftFront.getCurrentPosition()*r));
                        telemetry.update();
                }
                turnToTarget(0);
        }
        public void turnToTarget(double target) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                for(int i = 0; i < 5; i++) {
                        robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                        while (gyro.calcPID(target) != 0 && !isStopRequested()){
                                robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                                telemetry.addData("pidcalc", "%.3f", gyro.calcPID(target));
                                telemetry.update();
                        }
                        sleep(20);
                }
                robot.moveLateral(0,0,0,0);
        }
        public void driveArc(double target, double arcLength) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                arcLength += 1;
                double turnDir = Range.clip(-gyro.calcPID(target)*1000, -1, 1);
                double turnSpeed = 1 / arcLength;
                
                //correction
                turnSpeed *= 1.1 + 0.035*arcLength;
                
                robot.moveLateral(1, turnSpeed*2*turnDir, 0, 0);
                while(gyro.calcPID(target) != 0 && !isStopRequested()){
                        robot.moveLateral(1, turnSpeed*2*turnDir, 0, 0);
                }
                turnToTarget(target);
                robot.moveLateral(0,0,0,0);
        }
        public void liftPos(int pos) {
                robot.lift.setTargetPosition(pos);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
                while(robot.lift.isBusy()){}
        }
        public void driveStraight(double target, double speed, int reverse) {
                robot.moveLateral(speed, -gyro.calcPID(target)*reverse, 0, 0);
        }
        public void straifStraight(double target, double speed, int reverse) {
                robot.moveLateral(0, -gyro.calcPID(target)*reverse, speed, 0);
        }
        public void scanColor() {
                int cnt = 0;
                robot.color.enableLed(false);
                while(robot.color.red() > 35 && cnt < 2){
                        colorOffset += 8;
                        cnt += 1;
                        moveToPosition(-8*redSide, 0.6);
                        telemetry.addData("combined", "%d", robot.color.argb());
                        telemetry.addData("red", "%d", robot.color.red());
                        telemetry.addData("green", "%d", robot.color.green());
                        telemetry.addData("blue", "%d", robot.color.blue());
                        telemetry.update();
                }
                if(cnt == 0)
                        colorOffset2 = -32;
        }
}