# 도미노 연습

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    
    ON, OFF = 1, 0
    
    from DSR_ROBOT2 import (
        DR_BASE,
        DR_TOOL,
        DR_MV_MOD_REL,
        DR_MV_MOD_ABS,
        set_desired_force,
        release_force,
        DR_FC_MOD_REL,
        task_compliance_ctrl,
        release_compliance_ctrl,
        check_force_condition,
        check_position_condition,
        DR_AXIS_Z,
        set_digital_output,
        posx,
        posj,
        movel,
        movej,
        set_tool,
        set_tcp,        
    )    
    
    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        
    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)      
    
    Jready = [0, 0, 90, 0, 90, 0]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    start_pos = [349.49, -189.34, 280, 0, 180, 0]
    i = 0
            
    while True: 
        movej(Jready, vel=50, acc=50)
        grip()
        movel(pos=posx([0, 0, 100, 0, 0, 0]), vel=50, acc=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(pos=posx(start_pos), vel=50, acc=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        
        # if i % 3 == 0:
        
        movel(pos=posx([30, -80, 0, 0, 0, 0]), vel= 50, acc= 50, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(pos=posx([0, 0, -36, 0, 0, 0]), vel= 50, acc= 50, ref=DR_BASE, mod=DR_MV_MOD_REL)
                
        task_compliance_ctrl()
        set_desired_force(fd=[0, 15, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
                
        time.sleep(10)
                
        release_compliance_ctrl()
        
        movel(pos=posx([0, 0, 36, 0, 0, 0]), vel= 50, acc= 50, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(pos=posx([0, 70, 0, 0, 0, 0]), vel= 50, acc= 50, ref=DR_BASE, mod=DR_MV_MOD_REL)
        
        release()
        movel(pos=posx([0, 0, -50, 0, 0, 0]), vel= 50, acc= 50, ref=DR_BASE, mod=DR_MV_MOD_REL)
        grip()
        time.sleep(3)
        
if __name__ == "__main__":
    main()    
                
        