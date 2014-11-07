/*
 * Copyright (c) 2013, Kevin DeMarco
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <rqt_p2os/p2os.h>

#include <QMessageBox>
#include <QPainter>

using std::cout;
using std::endl;

namespace rqt_p2os {

     p2os::p2os()
          : rqt_gui_cpp::Plugin()
          , widget_(0)
     {
          setObjectName("p2os");
     }

     void p2os::initPlugin(qt_gui_cpp::PluginContext& context)
     {
          widget_ = new QWidget();
          ui_.setupUi(widget_);
		  checked = false;

          if (context.serialNumber() > 1)
          {
               widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
          }
          context.addWidget(widget_);

          // Enable / disable motors
          connect(ui_.motorButton, SIGNAL(clicked()), this, SLOT(onMotorEnable()));
          connect(this, SIGNAL(valueChanged(double)), this, SLOT(setValue(double)));
          connect(this, SIGNAL(valueChangedMotorState(bool)), this, SLOT(setValueMotorState(bool)));
          
          // Create publish and subscriber example
          this->publisher_ = getNodeHandle().advertise<p2os_msgs::MotorState>("/cmd_motor_state", 1000);
          this->subscriber_ = getNodeHandle().subscribe<p2os_msgs::BatteryState>("/battery_state", 1, &p2os::callbackBattery, this);  

			this->motor_state_sub_ = getNodeHandle().subscribe<p2os_msgs::MotorState>("/motor_state", 1, &p2os::callbackMotorState, this);  


     }
     

     bool p2os::eventFilter(QObject* watched, QEvent* event)
     {
          return QObject::eventFilter(watched, event);
     }

     void p2os::shutdownPlugin()
     {
          subscriber_.shutdown();
          publisher_.shutdown();
     }
     
     void p2os::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
     {

     }

     void p2os::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
     {
          
     }

	void p2os::setValueAndEmit(double v)
	{
		// if the value has changed
		//printf("The set value for %f\n", v);
		if (v != mem_value)
		{
		    mem_value = v;
		    emit valueChanged(v);
		}
	}

	void p2os::setValueAndEmitMotorState(bool v)
	{
		if (v != checked)
		{
		    checked = v;
		    emit valueChangedMotorState(v);
		}
	}

	void p2os::setValue(double v)
	{
		// if the value has changed
		//printf("The new value for %f\n", v);
		mem_value = v;

		ui_.batteryBar->setValue(mem_value);
	}

	void p2os::setValueMotorState(bool v)
	{
		// if the value has changed
		//printf("The new value for %f\n", v);
		checked = v;
		if(checked){
        	ui_.motorButton->setText("Disable Motors"); 
		}else{
            ui_.motorButton->setText("Enable Motors"); 
		}
	}
     
     void p2os::onMotorEnable()
     {
		  p2os_msgs::MotorState motor;
		  if (checked){
			checked = false;
            ui_.motorButton->setText("Enable Motors"); 
			motor.state = 0;
			this->publisher_.publish(motor);
		  }else{
			checked = true;
            ui_.motorButton->setText("Disable Motors"); 
			motor.state = 1;
			this->publisher_.publish(motor);
		  }

			
           
     }


     void p2os::callbackBattery(const p2os_msgs::BatteryStateConstPtr& msg)
     {
		double voltage = msg->voltage;
		double percent = (1.0-(14.0-voltage)/4.0) *100;
		//ROS_INFO("voltage %f", voltage);
		setValueAndEmit(percent);
     }     

     void p2os::callbackMotorState(const p2os_msgs::MotorStateConstPtr& msg)
     {
		int state = msg->state;

		setValueAndEmitMotorState(state);
     }  
}

PLUGINLIB_EXPORT_CLASS(rqt_p2os::p2os, rqt_gui_cpp::Plugin)
