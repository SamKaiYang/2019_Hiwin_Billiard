#test lauch file

roslaunch ROS_Socket Hiwin_RT605_Strategy_test.launch 

#更新時間 20190711
#新增
1. 吸盤功能,待測試
#注意
1. 主程式封包處理完成,尚須delay(目前測試0.05ms即可),跳動作問題也解決
2. 手臂電腦主程式Socket執行過久或傳送太久會當掉,已解決

#程式注意
1. 呼叫手臂動作如:point_data,Arm_Mode,Speed_Mode
2. 使用以上功能須最後輸入Arm_Mode,因為呼叫他才會發送指令
若先呼叫Arm_Mode再呼叫point_data則不會存入點位
3. 上述注意事項可參考Hiwin_RT605_Strategy_test.py中的寫法
4. ex:
ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)

ArmTask.Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                
