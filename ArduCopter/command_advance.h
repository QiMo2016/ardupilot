/*
 * command_advance.h
 *
 *  Created on: 2017年6月13日
 *      Author: moqi
 */

#ifndef ARDUCOPTER_COMMAND_ADVANCE_H_
#define ARDUCOPTER_COMMAND_ADVANCE_H_
/// @class    AP_Mission
/// @brief    Object managing Mission
class COMMAND_ADVANCE {

public:

	   // 指令内容
	    // 暂定之后会对指令进行分类，对原有指令进行引用，新的高级指令重新定义结构体


    // 指令运行状态
    enum command_adv_state {
        COMMAND_STOPPED=0,// 停止指令运行
		COMMAND_RUNNING=1,// 有指令正在运行
        COMMAND_COMPLETE=2,// 指令运行已完成
    };

    // 指令标志位
    struct COMMAND_ADV_Flags {
    	command_adv_state state;
        uint8_t new_commmand  : 1; // 有新的指令时设置为1
        uint8_t resolution_command : 1; // 需要进行指令分解时设置为1
    } _flags;

    void update();//指令更新
    void run();//指令运行
    void verify();//指令确认

private:
    void resolution();// 指令分解
    void save();// 指令存储

};





#endif /* ARDUCOPTER_COMMAND_ADVANCE_H_ */
