# recorder.py: class implementation to record baxter data during record and playback
# Requirements: these programs assume that the hook gripper is connected
# Author: Nishanth Koganti
# Date: 2015/08/24
# Source: baxter_examples/src/baxter_examples/recorder.py

# TODO:
# 1) if required consider implementing class with ros bag

# import external libraries
import rospy
import baxter_interface

# joint recorder class
class JointRecorder(object):
    def __init__(self, filename, rate, mode):
        """Records joint data to a file at a specified rate."""

        # initialize class variables
        self._filename = filename
        self._filename1 = filename + 'JA'
        self._filename2 = filename + 'JT'
        self._filename3 = filename + 'EE'

        self._mode = mode
        self._done = False
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()

        # robot variables initialization
        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")

        # get joint names
        self._joints_left = self._limb_left.joint_names()
        self._joints_right = self._limb_right.joint_names()

        # create files depending on mode variable
        # 0 is record mode and 1 is playback mode
        if self._mode == 0 and self._filename:
            self._f = open(self._filename,'w')

            # write names of self._f columns to SELF._Files 1,2,3
            self._f.write('time,')
            self._f.write(','.join([j for j in self._joints_left]) + ',')
            self._f.write(','.join([j for j in self._joints_right]) + '\n')

        if self._mode == 1 and self._filename1 and self._filename2:
            # open the files
            self._f1 = open(self._filename1,'w')
            self._f2 = open(self._filename2,'w')
            self._f3 = open(self._filename3,'w')

            # write the names of columns to Files 1,2,3
            self._f1.write('time,')
            self._f1.write(','.join([j for j in self._joints_left]) + ',')
            self._f1.write(','.join([j for j in self._joints_right]) + '\n')

            self._f2.write('time,')
            self._f2.write(','.join([j for j in self._joints_left]) + ',')
            self._f2.write(','.join([j for j in self._joints_right]) + ',')
            self._f2.write(','.join([j for j in self._joints_left]) + ',')
            self._f2.write(','.join([j for j in self._joints_right]) + '\n')

            self._f3.write('time,')
            self._f3.write('poselinear_left_x,poselinear_left_y,poselinear_left_z,')
            self._f3.write('poseangle_left_x,poseangle_left_y,poseangle_left_z,poseangle_left_w,')
            self._f3.write('poselinear_right_x,poselinear_right_y,poselinear_right_z,')
            self._f3.write('poseangle_right_x,poseangle_right_y,poseangle_right_z,poseangle_right_w,')
            self._f3.write('twistlinear_left_x,twistlinear_left_y,twistlinear_left_z,')
            self._f3.write('twistangular_left_x,twistangular_left_y,twistangular_left_z,')
            self._f3.write('twistlinear_right_x,twistlinear_right_y,twistlinear_right_z,')
            self._f3.write('twistangular_right_x,twistangular_right_y,twistangular_right_z,')
            self._f3.write('wrenchforce_left_x,wrenchforce_left_y,wrenchforce_left_z,')
            self._f3.write('wrenchtorque_left_x,wrenchtorque_left_y,wrenchtorque_left_z,')
            self._f3.write('wrenchforce_right_x,wrenchforce_right_y,wrenchforce_right_z,')
            self._f3.write('wrenchtorque_right_x,wrenchtorque_right_y,wrenchtorque_right_z\n')

    def _time_stamp(self):
        # useful for generating the time stamp
        return rospy.get_time() - self._start_time

    def stop(self):
        # function to stop recording
        """Stop recording."""
        self._done = True

    def done(self):
        # function for clean exit
        """Return whether or not recording is done."""
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def setTime(self):
        # function to set start time for time stamps in files
        self._start_time = rospy.get_time()


    def recordOnce(self):
        """Records current joint positions to csv file"""
        if self._mode == 0 and self._filename:
            # obtain joint angles, velocities and torques
            angles_left = [self._limb_left.joint_angle(j) for j in self._joints_left]
            angles_right = [self._limb_right.joint_angle(j) for j in self._joints_right]

            # write joint angle data to file 1
            self._f.write("%f," % (self._time_stamp(),))
            self._f.write(','.join([str(x) for x in angles_left]) + ',')
            self._f.write(','.join([str(x) for x in angles_right]) + '\n')

            self._rate.sleep()

        if self._mode == 1 and self._filename1 and self._filename2:
            # obtain Joint angles, velocities and torques
            angles_left = [self._limb_left.joint_angle(j) for j in self._joints_left]
            angles_right = [self._limb_right.joint_angle(j) for j in self._joints_right]
            velocities_left = [self._limb_left.joint_velocity(j) for j in self._joints_left]
            velocities_right = [self._limb_right.joint_velocity(j) for j in self._joints_right]
            torques_left = [self._limb_left.joint_effort(j) for j in self._joints_left]
            torques_right = [self._limb_right.joint_effort(j) for j in self._joints_right]

            # obtain end-effector position, velocity and force
            endpose_left = self._limb_left.endpoint_pose()
            endpose_right = self._limb_right.endpoint_pose()
            if all('position' in k for k in (endpose_left.keys(), endpose_right.keys())):
                endvelocity_left = self._limb_left.endpoint_velocity()
                endvelocity_right = self._limb_right.endpoint_velocity()
                endeffort_left = self._limb_left.endpoint_effort()
                endeffort_right = self._limb_right.endpoint_effort()

                # write joint angle data to file 1
                self._f1.write("%f," % (self._time_stamp(),))
                self._f1.write(','.join([str(x) for x in angles_left]) + ',')
                self._f1.write(','.join([str(x) for x in angles_right]) + '\n')

                # write joint velocity, torque data to file 2
                self._f2.write("%f," % (self._time_stamp(),))
                self._f2.write(','.join([str(x) for x in velocities_left]) + ',')
                self._f2.write(','.join([str(x) for x in velocities_right]) + ',')
                self._f2.write(','.join([str(x) for x in torques_left]) + ',')
                self._f2.write(','.join([str(x) for x in torques_right]) + '\n')

                # write the end-effector information to file 3
                self._f3.write("%f," % (self._time_stamp(),))
                self._f3.write("%f,%f,%f," % (endpose_left['position'].x,endpose_left['position'].y,endpose_left['position'].z))
                self._f3.write("%f,%f,%f,%f," % (endpose_left['orientation'].x,endpose_left['orientation'].y,endpose_left['orientation'].z,endpose_left['orientation'].w))
                self._f3.write("%f,%f,%f," % (endpose_right['position'].x,endpose_right['position'].y,endpose_right['position'].z))
                self._f3.write("%f,%f,%f,%f," % (endpose_right['orientation'].x,endpose_right['orientation'].y,endpose_right['orientation'].z,endpose_right['orientation'].w))

                self._f3.write("%f,%f,%f," % (endvelocity_left['linear'].x,endvelocity_left['linear'].y,endvelocity_left['linear'].z))
                self._f3.write("%f,%f,%f," % (endvelocity_left['angular'].x,endvelocity_left['angular'].y,endvelocity_left['angular'].z))
                self._f3.write("%f,%f,%f," % (endvelocity_right['linear'].x,endvelocity_right['linear'].y,endvelocity_right['linear'].z))
                self._f3.write("%f,%f,%f," % (endvelocity_right['angular'].x,endvelocity_right['angular'].y,endvelocity_right['angular'].z))

                self._f3.write("%f,%f,%f," % (endeffort_left['force'].x,endeffort_left['force'].y,endeffort_left['force'].z))
                self._f3.write("%f,%f,%f," % (endeffort_left['torque'].x,endeffort_left['torque'].y,endeffort_left['torque'].z))
                self._f3.write("%f,%f,%f," % (endeffort_right['force'].x,endeffort_right['force'].y,endeffort_right['force'].z))
                self._f3.write("%f,%f,%f\n" % (endeffort_right['torque'].x,endeffort_right['torque'].y,endeffort_right['torque'].z))
            else:
                print("Read Error!")

            self._rate.sleep()

    def record(self):
        """Records current joint positions to csv file"""
        if self._mode == 0 and self._filename:
             while not self._done:
                # obtain Joint angles, velocities and torques
                angles_left = [self._limb_left.joint_angle(j) for j in self._joints_left]
                angles_right = [self._limb_right.joint_angle(j) for j in self._joints_right]

                # write joint angle data to file 1
                self._f.write("%f," % (self._time_stamp(),))
                self._f.write(','.join([str(x) for x in angles_left]) + ',')
                self._f.write(','.join([str(x) for x in angles_right]) + '\n')

                # sleep
                self._rate.sleep()

        if self._mode == 1 and self._filename1 and self._filename2:
            while not self._done:
                # obtain Joint angles, velocities and torques
                angles_left = [self._limb_left.joint_angle(j) for j in self._joints_left]
                angles_right = [self._limb_right.joint_angle(j) for j in self._joints_right]
                velocities_left = [self._limb_left.joint_velocity(j) for j in self._joints_left]
                velocities_right = [self._limb_right.joint_velocity(j) for j in self._joints_right]
                torques_left = [self._limb_left.joint_effort(j) for j in self._joints_left]
                torques_right = [self._limb_right.joint_effort(j) for j in self._joints_right]

                # obtain end-effector position, velocity and force
                endpose_left = self._limb_left.endpoint_pose()
                endpose_right = self._limb_right.endpoint_pose()
                endvelocity_left = self._limb_left.endpoint_velocity()
                endvelocity_right = self._limb_right.endpoint_velocity()
                endeffort_left = self._limb_left.endpoint_effort()
                endeffort_right = self._limb_right.endpoint_effort()

                # write joint angle data to file 1
                self._f1.write("%f," % (self._time_stamp(),))
                self._f1.write(','.join([str(x) for x in angles_left]) + ',')
                self._f1.write(','.join([str(x) for x in angles_right]) + '\n')

                # write joint velocity, torque data to file 2
                self._f2.write("%f," % (self._time_stamp(),))
                self._f2.write(','.join([str(x) for x in velocities_left]) + ',')
                self._f2.write(','.join([str(x) for x in velocities_right]) + ',')
                self._f2.write(','.join([str(x) for x in torques_left]) + ',')
                self._f2.write(','.join([str(x) for x in torques_right]) + '\n')

                # write the end-effector information to file 3
                self._f3.write("%f," % (self._time_stamp(),))
                self._f3.write("%f,%f,%f," % (endpose_left['position'].x,endpose_left['position'].y,endpose_left['position'].z))
                self._f3.write("%f,%f,%f,%f," % (endpose_left['orientation'].x,endpose_left['orientation'].y,endpose_left['orientation'].z,endpose_left['orientation'].w))
                self._f3.write("%f,%f,%f," % (endpose_right['position'].x,endpose_right['position'].y,endpose_right['position'].z))
                self._f3.write("%f,%f,%f,%f," % (endpose_right['orientation'].x,endpose_right['orientation'].y,endpose_right['orientation'].z,endpose_right['orientation'].w))

                self._f3.write("%f,%f,%f," % (endvelocity_left['linear'].x,endvelocity_left['linear'].y,endvelocity_left['linear'].z))
                self._f3.write("%f,%f,%f," % (endvelocity_left['angular'].x,endvelocity_left['angular'].y,endvelocity_left['angular'].z))
                self._f3.write("%f,%f,%f," % (endvelocity_right['linear'].x,endvelocity_right['linear'].y,endvelocity_right['linear'].z))
                self._f3.write("%f,%f,%f," % (endvelocity_right['angular'].x,endvelocity_right['angular'].y,endvelocity_right['angular'].z))

                self._f3.write("%f,%f,%f," % (endeffort_left['force'].x,endeffort_left['force'].y,endeffort_left['force'].z))
                self._f3.write("%f,%f,%f," % (endeffort_left['torque'].x,endeffort_left['torque'].y,endeffort_left['torque'].z))
                self._f3.write("%f,%f,%f," % (endeffort_right['force'].x,endeffort_right['force'].y,endeffort_right['force'].z))
                self._f3.write("%f,%f,%f\n" % (endeffort_right['torque'].x,endeffort_right['torque'].y,endeffort_right['torque'].z))

                self._rate.sleep()
