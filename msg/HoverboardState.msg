# status codes
uint16 STATUS_ENABLED              =1
uint16 STATUS_CTRL_MODE            =2
uint16 STATUS_LEFT_MOTOR_ERR       =4
uint16 STATUS_RIGHT_MOTOR_ERR      =8

uint16 STATUS_PCB_TEMP_WARN        =16
uint16 STATUS_PCB_TEMP_ERR         =32
uint16 STATUS_LEFT_MOTOR_TEMP_ERR  =64
uint16 STATUS_RIGHT_MOTOR_TEMP_ERR =128

uint16 STATUS_CONN_TIMEOUT         =256
uint16 STATUS_ADC_TIMEOUT          =512
uint16 STATUS_GEN_TIMEOUT          =1024

uint16 STATUS_BATTERY_DEAD         =4096
uint16 STATUS_BATTERY_L1           =8192
uint16 STATUS_BATTERY_L2           =16384

# connection states
uint8 HOVERBOARD_CONNECTION_STATE_DISCONNECTED=0
#uint8 HOVERBOARD_CONNECTION_STATE_WAITING_FOR_FW=1
uint8 HOVERBOARD_CONNECTION_STATE_CONNECTED_ONEWAY=2
uint8 HOVERBOARD_CONNECTION_STATE_CONNECTED=3

uint8 connection_state
#rad/s
float64 cmdR
float64 cmdL
#rad/s
float64 speedR_meas
float64 speedL_meas
#rad
float64 wheelR_cnt
float64 wheelL_cnt
float64 currR_meas
float64 currL_meas
float64 motorR_temp
float64 motorL_temp
float64 batVoltage
float64 boardTemp
uint16 status

