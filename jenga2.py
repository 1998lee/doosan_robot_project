# 삼각형으로 쌓기 실패

import rclpy
import DR_init

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
    i = 0
    cnt = 0
            
    while True: 
        movej(Jready, vel=50, acc=50)
        grip()
        
        movel(pos=posx([0, 0, 100, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        
        task_compliance_ctrl()
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
            
        release_compliance_ctrl()
        
        movel(pos=posx([0, 0, -10, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        release()
        movel(pos=posx([0, 0, 30, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        grip()
        
        movel(pos=posx([0, 0, -100, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        movel(pos=posx([150, 0, 0, 0, 0, 90]), vel= 50, acc= 50, ref=DR_BASE, mod=DR_MV_MOD_REL)
         
        if i % 6 == 0:
            print(cnt)
            movel(pos=posx([0, 34.15, 100 - 15 * cnt, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            
            task_compliance_ctrl()
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass      
            
            release_compliance_ctrl()
            release()
            
            movel(pos=posx([0, 0, -100, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            i += 1
        
        elif i % 6 == 1:
            print(cnt)
            movel(pos=posx([-24.15, -24.15, 100 - 15*cnt, 0, 0, -60]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            
            task_compliance_ctrl()
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
        
            release_force()   
            release_compliance_ctrl()
            release()
            
            movel(pos=posx([0, 0, -100, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            i += 1
        
        elif i % 6 == 2:
            print(cnt)
            movel(pos=posx([24.15, -24.15, 100 - 15*cnt, 0, 0, 60]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            
            task_compliance_ctrl()
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
        
            release_force()   
            release_compliance_ctrl()
            release()
            
            movel(pos=posx([0, 0, -100, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            i += 1
            cnt += 1
            
        elif i % 6 == 3:
            print(cnt)
            movel(pos=posx([0, -21.65, 0, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(pos=posx([0, -34.15, 100 - 15*cnt, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            
            task_compliance_ctrl()
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
        
            release_force()   
            release_compliance_ctrl()
            release()
            
            movel(pos=posx([0, 0, -100, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            i += 1
            
        elif i % 6 == 4:
            movel(pos=posx([0, -21.65, 0, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(pos=posx([-24.15, 24.15, 100 - 15*cnt, 0, 0, 60]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            
            task_compliance_ctrl()
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
        
            release_force()   
            release_compliance_ctrl()
            release()
            
            movel(pos=posx([0, 0, -100, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            i += 1
            
        elif i % 6 == 5:
            movel(pos=posx([0, -21.65, 0, 0, 0, 0]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(pos=posx([24.15, 24.15, 100 - 15*cnt, 0, 0, -60]), vel= 50, acc= 50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            
            task_compliance_ctrl()
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
        
            release_force()   
            release_compliance_ctrl()
            release()
            
            movel(pos=posx([0, 0, -100, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            i += 1
            cnt += 1
             
if __name__ == "__main__":
    main()    
                
        