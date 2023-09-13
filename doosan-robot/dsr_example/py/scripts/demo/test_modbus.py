#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 

ROBOT_SYSTEM_VIRTUAL = 1
ROBOT_SYSTEM_REAL = 0
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state           : %d" % (msg.robot_state))
        print("  robot_state_str       : %s" % (msg.robot_state_str))
        print("  actual_mode           : %d" % (msg.actual_mode))
        print("  actual_space          : %d" % (msg.actual_space))
        print("  current_posj          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        print("  current_velj          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_velj[0],msg.current_velj[1],msg.current_velj[2],msg.current_velj[3],msg.current_velj[4],msg.current_velj[5]))
        print("  joint_abs             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.joint_abs[0],msg.joint_abs[1],msg.joint_abs[2],msg.joint_abs[3],msg.joint_abs[4],msg.joint_abs[5]))
        print("  joint_err             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.joint_err[0],msg.joint_err[1],msg.joint_err[2],msg.joint_err[3],msg.joint_err[4],msg.joint_err[5]))
        print("  target_posj           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.target_posj[0],msg.target_posj[1],msg.target_posj[2],msg.target_posj[3],msg.target_posj[4],msg.target_posj[5]))
        print("  target_velj           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.target_velj[0],msg.target_velj[1],msg.target_velj[2],msg.target_velj[3],msg.target_velj[4],msg.target_velj[5]))    
        print("  current_posx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))
        print("  current_velx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_velx[0],msg.current_velx[1],msg.current_velx[2],msg.current_velx[3],msg.current_velx[4],msg.current_velx[5]))
        print("  task_err              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.task_err[0],msg.task_err[1],msg.task_err[2],msg.task_err[3],msg.task_err[4],msg.task_err[5]))
        print("  solution_space        : %d" % (msg.solution_space))
        sys.stdout.write("  rotation_matrix       : ")
        for i in range(0 , 3):
            sys.stdout.write(  "dim : [%d]"% i)
            sys.stdout.write("  [ ")
            for j in range(0 , 3):
                sys.stdout.write("%d " % msg.rotation_matrix[i].data[j])
            sys.stdout.write("] ")
        print ##end line
        print("  dynamic_tor           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.dynamic_tor[0],msg.dynamic_tor[1],msg.dynamic_tor[2],msg.dynamic_tor[3],msg.dynamic_tor[4],msg.dynamic_tor[5]))
        print("  actual_jts            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_jts[0],msg.actual_jts[1],msg.actual_jts[2],msg.actual_jts[3],msg.actual_jts[4],msg.actual_jts[5]))
        print("  actual_ejt            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_ejt[0],msg.actual_ejt[1],msg.actual_ejt[2],msg.actual_ejt[3],msg.actual_ejt[4],msg.actual_ejt[5]))
        print("  actual_ett            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_ett[0],msg.actual_ett[1],msg.actual_ett[2],msg.actual_ett[3],msg.actual_ett[4],msg.actual_ett[5]))
        print("  sync_time             : %7.3f" % (msg.sync_time))
        print("  actual_bk             : %d %d %d %d %d %d" % (msg.actual_bk[0],msg.actual_bk[1],msg.actual_bk[2],msg.actual_bk[3],msg.actual_bk[4],msg.actual_bk[5]))
        print("  actual_bt             : %d %d %d %d %d " % (msg.actual_bt[0],msg.actual_bt[1],msg.actual_bt[2],msg.actual_bt[3],msg.actual_bt[4]))
        print("  actual_mc             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_mc[0],msg.actual_mc[1],msg.actual_mc[2],msg.actual_mc[3],msg.actual_mc[4],msg.actual_mc[5]))
        print("  actual_mt             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_mt[0],msg.actual_mt[1],msg.actual_mt[2],msg.actual_mt[3],msg.actual_mt[4],msg.actual_mt[5]))

        #print digital i/o
        sys.stdout.write("  ctrlbox_digital_input : ")
        for i in range(0 , 16):
            sys.stdout.write("%d " % msg.ctrlbox_digital_input[i])
        print ##end line
        sys.stdout.write("  ctrlbox_digital_output: ")
        for i in range(0 , 16):
            sys.stdout.write("%d " % msg.ctrlbox_digital_output[i])
        print
        sys.stdout.write("  flange_digital_input  : ")
        for i in range(0 , 6):
            sys.stdout.write("%d " % msg.flange_digital_input[i])
        print
        sys.stdout.write("  flange_digital_output : ")
        for i in range(0 , 6):
            sys.stdout.write("%d " % msg.flange_digital_output[i])
        print
        #print modbus i/o
        sys.stdout.write("  modbus_state          : " )
        if len(msg.modbus_state) > 0:
            for i in range(0 , len(msg.modbus_state)):
                sys.stdout.write("[" + msg.modbus_state[i].modbus_symbol)
                sys.stdout.write(", %d] " % msg.modbus_state[i].modbus_value)
        print

        print("  access_control        : %d" % (msg.access_control))
        print("  homming_completed     : %d" % (msg.homming_completed))
        print("  tp_initialized        : %d" % (msg.tp_initialized))
        print("  mastering_need        : %d" % (msg.mastering_need))
        print("  drl_stopped           : %d" % (msg.drl_stopped))
        print("  disconnected          : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    
  
if __name__ == "__main__":
    rospy.init_node('dsr_simple_test_py')
    rospy.on_shutdown(shutdown)
    #set_robot_mode(ROBOT_MODE_MANUAL)
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    rospy.loginfo(str(MODBUS_REGISTER_TYPE_DISCRETE_INPUTS))
    ip = "192.168.137.150"#string값
    port = 502
    add_modbus_signal(ip, port, "ACT", DR_MODBUS_REG_OUTPUT, 0, 0) #modbus signal ACT 등록 : action request & gripper option
    add_modbus_signal(ip, port, "WID", DR_MODBUS_REG_OUTPUT, 1, 0) # modbus signal WID등록 : Gripper Position
    add_modbus_signal(ip, port, "FOR", DR_MODBUS_REG_OUTPUT, 2, 0) # modbus signal FOR 등록 : Gripper Force & Gripper Speed
    wait(1)
    pos_ready = [2, 0.2, 105, -181, -77, -45]
    
    movej(pos_ready, 60, 30)
    set_modbus_output("ACT", 2304) # action request = 0x09 & gripper options = 0x00 / Gripper 활성화 신호와 Gripper 이동 요청 명령을 1로 set
    set_modbus_output("WID", 0) # Gripper Position 을 최소 위치로 변경 (Open) / Gripper options2 = 0x00 & gripper position = 0x00
    set_modbus_output("FOR", 65535) # Speed = 0xFF / Force = 0xFF
    
    wait(2)
    pos_1 = [5.477921009063721, 15.57923698425293, 117.46717071533203, -187.10777282714844, -47.733306884765625, -44.358455657958984]
    movej(pos_1, 60, 30)
    set_modbus_output("ACT", 2304) # action request = 0x09 & gripper options = 0x00 / Gripper 활성화 신호와 Gripper 이동 요청 명령을 1로 set
    set_modbus_output("WID", 255) # Gripper Position 을 최소 위치로 변경 (Open) / Gripper options2 = 0x00 & gripper position = 0x00
    set_modbus_output("FOR", 65535) # Speed = 0xFF / Force = 0xFF
    wait(2)
    x = [0, 0, 100, 0, 0, 0]
    movel(x, 60, 30, mod = DR_MV_MOD_REL)
    
    pos_2 = [-5.254105091094971, -7.432934284210205, 129.5436248779297, -186.1969757080078, -57.51133346557617, -56.630428314208984]
    movej(pos_2, 60, 30)

    pos_3 = [-7.5329999923706055, -0.23423683643341064, 134.00205993652344, -186.6237335205078, -47.584686279296875, -56.85720443725586]
    movej(pos_3, 30, 30)
    

    set_modbus_output("WID", 0)
    
    while not rospy.is_shutdown():
        pass

    print('good bye!')
