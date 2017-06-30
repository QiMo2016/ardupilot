#include "Copter.h"

//
void Copter::command_update()
{
	advance.update();
}

//指令更新
void COMMAND_ADVANCE::update()
{
	//当无新指令并且无需要运行的指令时直接跳出
	if((_flags.new_commmand==0)||(_flags.state==COMMAND_COMPLETE))
	{
		return;
	}

	//指令分解
	if(_flags.resolution_command==1)
	{
		resolution();
		//save();
	}

	if(_flags.state==COMMAND_RUNNING)
	{
		//指令运行
		run();
	}

	//确认运行完成
	verify();
}

 void COMMAND_ADVANCE::resolution()
 {
	 switch(copter.cmda.ID)
	 {
	    case MAV_CMD_CONDITION_YAW:             // 115
	    	copter.cmda.repeat=copter.cmda.content.param1/179+2;//最后一次运行多余的角度,则
			copter.cmda.leave_yaw=copter.cmda.content.param1 - 179 *( copter.cmda.repeat-2);
	        break;
	 }
	 _flags.resolution_command=0;
 }

void COMMAND_ADVANCE::run() {
	switch (copter.cmda.ID) {
	case MAV_CMD_CONDITION_YAW:             // 115
		float yaw;
		if (copter.cmda.repeat > 2) {
			yaw = 179;
			copter.cmda.repeat--;
		} else if(copter.cmda.repeat > 1){
			yaw = copter.cmda.leave_yaw;
			copter.cmda.repeat--;
		}else{
			int i;
			i=copter.cmda.content.param1/360;
			yaw = 360*(i+1)-copter.cmda.content.param1;
			copter.cmda.content.param4=0;
			copter.cmda.repeat--;
		}

		copter.set_auto_yaw_look_at_heading(yaw, copter.cmda.content.param2,
				(int8_t) copter.cmda.content.param3,
				(uint8_t) copter.cmda.content.param4);
		break;
	}
}

void COMMAND_ADVANCE::verify()
{
	if(_flags.state==COMMAND_RUNNING)
	{
	_flags.state=COMMAND_STOPPED;
	}

	switch (copter.cmda.ID) {
	case MAV_CMD_CONDITION_YAW:             // 115
	    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
	    if (copter.auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
	    	copter.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
	    }

	    // check if we are within 2 degrees of the target heading
	    if (labs(wrap_180_cd(copter.ahrs.yaw_sensor-copter.yaw_look_at_heading)) <= 200) {
	    	_flags.state=COMMAND_RUNNING;
	    }
		break;
	}
	if(copter.cmda.repeat==0)
	{
	_flags.new_commmand=0;
	_flags.state=COMMAND_COMPLETE;
	}
}

 void Copter::mavlinktocmda(mavlink_command_long_t mavlink)
 {
	 cmda.content=mavlink;
	 advance._flags.resolution_command=1;
	 advance._flags.new_commmand=1;
	 advance._flags.state=advance.COMMAND_RUNNING;
 }
