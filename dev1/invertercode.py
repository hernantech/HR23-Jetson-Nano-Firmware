enabled = False\;
inverter_enabled = False
module_a_temperature = 0;
module_b_temperature = 0;
module_c_temperature = 0;
gate_drive_temperature = 0;
control_board_temperature = 0;
torque_command= "";

dc_bus_voltage = 0;
output_voltage = 0;
ab_voltage = 0;
bc_voltage = 0;
powered = False;
fault_active = False;

def Disable_Invertor(inverter_enabled):
   inverter_enabled = False;

def Enable_Invertor(inverter_enabled):
   inverter_enabled = True

def getDirection(direction):
   return direction

def getMaxTemperature(_self_):
   return control_board_temperature;
def reset():
   enabled = False;
   direction_counter = 0;
   torque_command = 0.0
   direction = 0;
   requested_direction = 0;
   inverter_enabled = 0;
   fault_active = 0;

def setTorqueCommand(tq_cmd):
	torque_command = tq_cmd;

def receiveCANMessage():
   print("received can message");

def isFaultActive():
	if (powered):
		return fault_active;
	else:
		return False;


def getMaxTemperature(self):
   max_temp = module_a_temperature

   if module_b_temperature > max_temp:
      max_temp = self.module_b_temperature
   if module_c_temperature > max_temp:
      max_temp = module_c_temperature
   if gate_drive_temperature > max_temp:
      max_temp = self.gate_drive_temperature
   return max_temp
def geMotorTempature():
    return 'Input Tempature Variable'
def initializeCANReceive():
    print("Can initialized");
def isFaultActive():
    if powered: ### input powered vairable
        return fault_active ### input fault_active variable
    else:
        return False

def sendCommandMessage():
    print("sending can Message");

def setDirection():
    ''' fill in the blank'''
def set_torque_command(tq_cmd):
    global torque_command
    torque_command = tq_cmd
def tick():
    if(enabled):
        direction = 2;
        inverter_enabled = False;
        sendCommandMessage();

        direction = 1;
        

