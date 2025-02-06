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
        amove_periodic,     
    )    
    
    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        
    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)      
    
    Jready = [0, 0, 90, 0, 90, 0]
    gear_pos= [[242.07, 54.79, 150, 72.29, 179.89, 71.96], [336.9, 8.73, 150, 65.6, -179.67, 65.61],
               [249.69, -50.98, 150, 54.14, -179.86, 53.98], [274.41, 5.28, 150, 127.39, 179.5, 127.52]]
    target_pos=[[513.78, 2.15, 150, 145.83, -179.42, 146.05], [608.25, -47.42, 150, 97.52, -179.5, 97.88],
                [600.96, 59.29, 150, 74.16, -179.43, 74.47], [573.69, 5.82, 150, 49.82, -179.14, 50.28]]    
    set_tool("Tool Weight")
    set_tcp("GripperDA_v1")
    
    for i in range(4):
        # go home
        movej(Jready, vel=50, acc=50)
        grip()
        
        # pick gear
        movel(gear_pos[i], vel=50, acc=50, ref=DR_BASE, mod=DR_MV_MOD_ABS) 
        movel(pos=posx([0,0,80,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        
        task_compliance_ctrl()
        set_desired_force(fd=[0,0,-10,0,0,0], dir=[0,0,1,0,0,0], mod=DR_FC_MOD_REL)
        
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        
        release_compliance_ctrl()
        
        # place gear
        if i != 3:
            movel(pos=posx([0,0,-10,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            release()
            movel(pos=posx([0,0,30,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            grip()
                
            movel(pos=posx([0,0,-150,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(target_pos[i], vel=50, acc=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movel(pos=posx([0,0,80,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                
            task_compliance_ctrl()
            set_desired_force(fd=[0,0,-10,0,0,0], dir=[0,0,1,0,0,0], mod=DR_FC_MOD_REL)
                
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
                
            release_compliance_ctrl()
            release()
            
            movel(pos=posx([0,0,-150,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            
        elif i == 3:
            movel(pos=posx([0,0,-10,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            release()
            movel(pos=posx([0,0,30,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            grip()
                
            movel(pos=posx([0,0,-150,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(target_pos[i], vel=50, acc=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movel(pos=posx([0,0,80,0,0,0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                
            task_compliance_ctrl()
            set_desired_force(fd=[0,0,-10,0,0,0], dir=[0,0,1,0,0,0], mod=DR_FC_MOD_REL)
            
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
        
            amove_periodic([0,0,0,0,0,30], period=30)
            
            while check_position_condition(DR_AXIS_Z, max=40):
                pass
        
            release_compliance_ctrl()
            release()
            
    movej(Jready, vel=50, acc=50)
    grip()
    
    rclpy.shutdown()          
            
if __name__ == "___main__":
        main()