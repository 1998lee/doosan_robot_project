import rclpy
import DR_init
import random

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
    
    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        
    def grip():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        
    def set_pose(start_pos, x_dif, y_dif, count, offset = 0):
        pos_list = []
        start_pos[0] += offset
        
        for i in range(count):
            if i == 0:
                pose = start_pos[:]
                pos_list.append(pose)
                for j in range(2):
                    pose = pose[:]
                    pose[0] += x_dif
                    pos_list.append(pose)
            elif i == 3:
                pose = start_pos[:]
                pose[1] += y_dif
                pos_list.append(pose)
                for j in range(2):
                    pose = pose[:]
                    pose[0] += x_dif
                    pos_list.append(pose)
            elif i == 6:
                pose = start_pos[:]
                pose[1] += y_dif*2
                pos_list.append(pose)
                for j in range(2):
                    pose = pose[:]
                    pose[0] += x_dif
                    pos_list.append(pose)                    
        return pos_list
    
    def place(pose):
        movel(pos=posx(pose), vel=50, acc=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(pos=posx([0, 0, 80, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        
        task_compliance_ctrl()
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        
        while True:
            if not check_force_condition(DR_AXIS_Z, min = 10):
                break
            
        release_force()
        release_compliance_ctrl()
        release()
    
    # 순방향이냐 역방향이냐
    mode = 0
       
    Jready = [0, 0, 90, 0, 90, 0]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
        
    movej(Jready, vel=50, acc=50)
    grip()
        
    long_index = 2
    mid_index = 1
    short_index = 0
    start_pos = [348.35, 101.46, 150, 29.34, 179.81, 29.7]
    x_dif = 50.0
    y_dif = -50.0
    pallet_offset = 150.0
    row = 3
    column = 3
    total_count = row*column
        
    pick_pos = set_pose(start_pos, x_dif, y_dif, total_count)
    target_pos = set_pose(start_pos, x_dif, y_dif, total_count, offset=pallet_offset)
    
    while True: 
        long_index = 2
        mid_index = 1
        short_index = 0    
        if mode == 0:
            for i in range(total_count):
                
                movel(pos=posx(pick_pos[i]), vel=50, acc=50, ref=DR_BASE, mod=DR_MV_MOD_ABS) 
                movel(pos=posx([0, 0, 80, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                
                task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                '''
                while True:
                    if not check_force_condition(DR_AXIS_Z, min=10):
                        long_check = check_position_condition(DR_AXIS_Z, max=65, min=55)
                        mid_check =  check_position_condition(DR_AXIS_Z, max=55, min=45)
                        short_check = check_position_condition(DR_AXIS_Z, max=45, min=35)
                        break
                '''
                while not check_force_condition(DR_AXIS_Z, max=5):
                    pass
                
                long_check = check_position_condition(DR_AXIS_Z, max=65, min=55)
                mid_check =  check_position_condition(DR_AXIS_Z, max=55, min=45)
                short_check = check_position_condition(DR_AXIS_Z, max=45, min=35)
                    
                release_force(0.5)
                release_compliance_ctrl()
                
                movel(pos=posx([0, 0, -20, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                release()    
                movel(pos=posx([0, 0, 40, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                grip()
                movel(pos=posx([0, 0, -150, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                
                if long_check == 0:
                    pose = target_pos[long_index]
                    place(pose)
                    long_index += 3
                    movel(pos=posx([0, 0, -150, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    grip()
                elif mid_check == 0:
                    pose = target_pos[mid_index]
                    place(pose)
                    mid_index += 3
                    movel(pos=posx([0, 0, -150, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    grip()
                elif short_check == 0:
                    pose = target_pos[short_index]
                    place(pose)
                    short_index += 3
                    movel(pos=posx([0, 0, -150, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    grip()
                    
                movej(Jready, vel=50, acc=50)
            mode = 1
                                    
        elif mode == 1:
            index_list = []
            i = 0
            
            while i < total_count:
                index = random.randint(0,8)
                if index not in index_list:
                    movel(pos=posx(target_pos[i]), vel=50, acc=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                    movel(pos=posx([0, 0, 80, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

                    while not check_force_condition(DR_AXIS_Z, max=5):
                        pass

                    release_compliance_ctrl()
                    
                    movel(pos=posx([0, 0, -20, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    release()    
                    movel(pos=posx([0, 0, 40, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    grip()
                    movel(pos=posx([0, 0, -150, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    
                    place(pick_pos[index])
                    movel(pos=posx([0, 0, -150, 0, 0, 0]), vel=50, acc=50, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    grip()
                    
                    movej(Jready, vel=50, acc=50)
                    index_list.append(index)
                    i += 1
                    
                else:
                    pass
                
            mode = 0
                    
if __name__ == '__main__':
    main()            
        

    
        
        
        
