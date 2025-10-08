from pyfirmata import ArduinoMega, util
import cv2
import numpy as np
import copy
import datetime
import pyfirmata

global int_error
global prev_error
global target_str

board = pyfirmata.ArduinoMega('COM13')
it = pyfirmata.util.Iterator(board)
it.start()
##########################################
f1 = open("D:/now_steer.txt", 'w')
f2 = open("D:/aim_steer.txt", 'w')
webcam = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
now = datetime.datetime.now().strftime("%d_%H-%M-%S")
# cv2.imshow('result',frame)

l_u_1 = (int(124),310)
l_l_1 = (int(34),420)
r_u_1 = (int(560),310)
r_l_1 = (int(640),420)
arr1 = [l_u_1, l_l_1, r_u_1, r_l_1]
m2pix = 540
pix2m = 1/540
halflane_m = 0.55
rearwheel2img = 1.35    # m

src_points_1 = np.array([l_u_1, l_l_1, r_u_1, r_l_1], dtype=np.float32)  # bev source
dst_points_1 = np.array([(0, 0), (0, 360), (630, 0), (630, 360)], dtype=np.float32)  # bev destination
M_1 = cv2.getPerspectiveTransform(src_points_1, dst_points_1)


def birdseye_view_transform(frame, M):  # bev 변환
    height = 360  # bev 최종 화면 크기, dst_point와 사이즈 맞출 것
    width = 630
    warped = cv2.warpPerspective(frame, M, (width, height), flags=cv2.INTER_LINEAR)

    return warped

################################################################

potentiometer = board.get_pin('a:1:i')

l_motor_1 = board.get_pin('d:7:p')
l_motor_2 = board.get_pin('d:6:p')

r_motor_1 = board.get_pin('d:4:p')
r_motor_2 = board.get_pin('d:3:p')

f_motor_1 = board.get_pin('d:8:p')
f_motor_2 = board.get_pin('d:9:p')

def main(v):
    global int_error
    global prev_error
    global target_str

    int_error = 0
    prev_error = 0
    target_str = 0

    potentiometer_value = potentiometer.read()
    # print("potentiometer_value", potentiometer_value)
    offset = 0.0127
    potentiometer_range_r = 0.0045
    potentiometer_range_degree_r = 20
    potentiometer_range_l = 0.0157
    potentiometer_range_degree_l = 30

    ret, frame = webcam.read()
    bev_frame = birdseye_view_transform(frame, M_1)  # bev 화면 검출
    
    lower_white_hls = np.array([0, 160, 0])         # 최소 색상(HLS)(밝은 회색)
    upper_white_hls = np.array([179, 255, 255])     # 최대 색상(HLS)(백색)
    # webcam = cv2.VideoCapture(0)
    # frame = cv2.imread('img2.png',cv2.IMREAD_COLOR)
    # cv2.imshow('result',frame)
    # print(bev_frame.shape[0], bev_frame.shape[1])
    # cv2.imwrite("logging.png", bev_frame)
    img_height = bev_frame.shape[0]
    img_width = bev_frame.shape[1]

    hls_frame = cv2.cvtColor(bev_frame, cv2.COLOR_BGR2HLS)    # HLS 화면으로 이미지 변환
    bi_hls_frame = cv2.inRange(hls_frame, lower_white_hls, upper_white_hls)  # 최소, 최대 색상 이외의 색상 0으로 변환
    canny_frame = cv2.Canny(bev_frame,150,150)



    # width_lane = 30
    x_val = np.zeros((img_height,img_width), dtype=np.uint8)


    for y in range(200, img_height-1):
        for x in range(img_width-40):
            if int(canny_frame[y][x]) != 0:
                for width in range(40):
                    if width < 20:
                        if int(canny_frame[y][x+width]) != 0:
                            pass
                    else:
                        if int(canny_frame[y][x+width]) != 0:
                            x_val[y][np.round(x + (width) / 2, 0).astype(np.int32)] = 255




    lines = cv2.HoughLinesP(x_val,1,np.pi/360,  threshold=30, minLineLength = 30, maxLineGap = 30)
    # print("Sfsdf", type(lines))

    try:
        if lines is not None:
            new_lines = lines.copy()

            i = 0
            
            while i < new_lines.shape[0]:
                [x1, y1, x2, y2] = new_lines[i][0]
                
                theta = np.arctan2(y2 - y1, x2 - x1)*180/np.pi
                critical_angle = 60
                if abs(theta) < 90:
                    if abs(theta) < critical_angle:
                        new_lines = np.delete(new_lines, i,0)
                    else:
                        cv2.line(bev_frame, (x1, y1), (x2, y2), (125, 0, 125), 3)
                        i += 1
                else:
                    if abs(theta) > 180 - critical_angle:
                        new_lines = np.delete(new_lines, i,0)
                    else:
                        cv2.line(bev_frame, (x1, y1), (x2, y2), (125, 0, 125), 3)
                        i += 1
        
                
                linesAvgX = (new_lines[:,0,0] + new_lines[:,0,2])/2

                linesRight =  new_lines[linesAvgX > 315]
                linesLeft =  new_lines[linesAvgX <= 315]  

                if len(linesRight) != 0 and len(linesLeft) !=0 :
                    final_lines = linesLeft
                else:
                    if linesRight.shape[0] > linesLeft.shape[0]:
                        final_lines = linesRight
                    else:
                        final_lines = linesLeft
                    # print("_____________:, ", final_lines.shape)
                if final_lines.shape[0] != 0:

                    k = 0
                    while k < final_lines.shape[0]:
                            [x1, y1, x2, y2] = final_lines[k][0]
                            cv2.line(bev_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                            k += 1
                        


                    cand_x = np.concatenate((final_lines[:, 0, 0], final_lines[:, 0, 2]))
                    cand_y = np.concatenate((final_lines[:, 0, 1], final_lines[:, 0, 3]))

                    line_fitting = np.polyfit(cand_y, cand_x, 1)
                    centerline_model = np.poly1d(line_fitting)
                    print(centerline_model)

                    topimg_height = 360 #pix
                    img_target_y = 200  #pixel y, 
                        # 1m = 542pix
                    rearwheel_y = topimg_height + rearwheel2img*m2pix
                    # target_y = (topimg_height - img_target_y)/0.06 + 1.25 #1.25m
                    # target_y = 0.5 #1.25m
                    # target_x = (centerline_model(target_y) - halflane_m)*pix2m
                    

                    rearwheel_x = (centerline_model(rearwheel_y) -315)*pix2m
                    img_target_x = centerline_model(img_target_y)
                    target_x = (img_target_x - 315) * pix2m
                    target_y = rearwheel2img + (topimg_height - img_target_y)*pix2m
                    print('--------------------------------------------')
                    
                    print('rearwheelx=',rearwheel_x)
                    if rearwheel_x >= 0 :
                        RightLane = True
                        # target_x = (centerline_model(target_y) -315)/600
                        target_x -= halflane_m
                        img_target_x -= halflane_m*600
                    
                    else:
                        RightLane = False
                        # target_x = (centerline_model(target_y) +315)/600
                        target_x += halflane_m
                        img_target_x += halflane_m*600
                    print('targetx=',target_x)
                    print('targety=',target_y)
                    wheel_base = 0.7
                    target_delta = (2*wheel_base*(target_x))/((target_x)**2+(target_y)**2)

                    alpha = 1  #steer ratio
                    target_str = target_delta*alpha*180/np.pi

                    # if img_target_x < -10:
                    #     img_target_x = 315
                    #     target_str = 0
                    # elif img_target_x > 630+100:
                    #     img_target_x = 315
                    #     target_str = 0

                    if target_str >= 0:
                        cv2.line(bev_frame, (315, 200), (int(315 + (target_str*3)), 200), (255, 0, 0), 20)
                    else:
                        cv2.line(bev_frame, (315, 200), (int(315 + (target_str*3)), 200), (255, 0, 0), 20)

                    cv2.circle(bev_frame, (int(img_target_x), img_target_y), 20, (0, 0, 255), -1)
                    # cv2.circle(bev_frame, (40, img_target_y), 20, (0, 255, 255), -1)                  # 0.94m = 510pix 
                    # cv2.circle(bev_frame, (550, img_target_y), 20, (0, 255, 255), -1)
    except:
        print('err')
        pass
                
    

    cv2.imshow('canny',canny_frame)
    # cv2.imshow('bi',bi_hls_frame)
    cv2.imshow('bev',bev_frame)
    cv2.imshow('result',x_val)
    
    def forward(v):
        l_motor_1.write(0)
        l_motor_2.write(v)
        r_motor_1.write(0)
        r_motor_2.write(v)

    def steering_control(aim_steering, eter_value, offset, dt):  # + --> left, -  --> right
        global int_error
        global prev_error
        try:
            
            print('potentio',potentiometer_value)
            if potentiometer_value < offset:
                steering_now = (potentiometer_value - offset) * potentiometer_range_degree_r / potentiometer_range_r    # degree
            elif potentiometer_value >= offset:
                steering_now = (potentiometer_value - offset) * potentiometer_range_degree_l / potentiometer_range_l
            print("steering_now :", steering_now)
            print("aim_steering :", aim_steering)
            error = aim_steering - steering_now
            

            print("error :", error)
            # print("error: ", error)
            P_gain = 0.02
            I_gain = 0
            D_gain = 0  

            # final_gain = 0.01

            diff_error = error - prev_error
            prev_error = error
            int_error += error

            steer = P_gain * error + D_gain * diff_error / dt + I_gain * int_error
            steer = np.clip(steer, -1, 1)

            print("steer: ", steer)
            if steer >= 0:
                f_motor_1.write(steer)
                f_motor_2.write(0)

            elif steer < 0:
                f_motor_1.write(0)
                f_motor_2.write(-steer)
        except:
            try :
                print('potentio : ',potentiometer_value)
            except :
                print('cannot read potentiometer val')
            pass

    # - -> right + -> left
    steering_control(-target_str, potentiometer_value, offset, 0.01)
    # steering_control(10, potentiometer_value, offset, 0.01)
    forward(v)
    
    key = cv2.waitKey(1)

    return key, bev_frame
    
if __name__ == '__main__':


    while webcam.isOpened():
        v =0.4
        key, bev = main(v)
        # video = cv2.VideoWriter("D:/" + str(now) + "kkk.avi", fourcc, 20.0, (bev.shape[1], bev.shape[0]))
        # video.write(bev)
        if key & 0xFF == ord('q'):
            # video.release()
            key, bev = main(0)
            break
    if not webcam.isOpened():
        print("Could not open webcam")
        
        exit()
        
    
    webcam.release()
    cv2.destroyAllWindows()