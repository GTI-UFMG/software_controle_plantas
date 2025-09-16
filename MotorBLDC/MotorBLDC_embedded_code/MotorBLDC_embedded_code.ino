#include <Arduino.h>

#include "control_system.h"

// Period for the message handler task in [ms].
// The period for the control task is defined in control_system.h
#define MESSAGE_HANDLER_TS 20

// Lengths of the data and command queues that 
// are used to implement inter-thread communication.
#define QUEUE_DATA_LENGTH 40
#define QUEUE_CMD_LENGTH 5

// Maximum number of command messages 
// expected to be in the serial buffer.
#define MAX_N_CMD_MSG 10

// Maximum number of elements in the
// set of pre-programmed reference values
#define MAX_REF_STEPS 32

// Maximum number of measured signals
#define MAX_N_MEASURED_SIGNALS 4 

// Maximum number of actuation signals
#define MAX_N_ACTUATION_SIGNALS 2

// Data Message structure:
typedef struct {
  float time;
  float output[2];
  float ctrl_input;
  float reference;
} msgData_package_t;

// List of possible commands (integer values):
#define START_CMD 0
#define STOP_CMD 1
#define STORE_REF_CMD 2
#define SET_REF_CMD 4
#define TEST_COMM 8
#define SET_TS 16
#define SET_CTRL_CODE 32
#define SET_CTRL_VARIABLE 36
#define SET_CTRL_SYS_PARAM 48
#define SET_DEADZONE_COMP 64

// Command message structure:
typedef struct {
  int cmd;
  float cmd_val1;
  float cmd_val2;
} msgCmd_package_t;

// Queues to exchange commands and data
// between Message Handler and Control  Task. 
QueueHandle_t queueData; // Control Task writes to this queue, and Message Handler reads from it.
QueueHandle_t queueCmd;  // Message Handler writes to this queue, and Control Task reads from it. 

TaskHandle_t MessageHandler_h;
TaskHandle_t ControlTask_h;

// Message Handler:
// This task should take care of receiving messages from the Python GUI,
// and sending data to be displayed in the Python GUI,
// over the serial port.
void MessageHandler(void *pvParameters)
{
  msgData_package_t msgData_package;
  msgCmd_package_t msgCmd_package;

  char cmd_buffer[MAX_N_CMD_MSG*sizeof(msgCmd_package_t)];
  
  unsigned char c;
  unsigned char cmd_buffer_pos = 0;

  TickType_t xLastWakeTime;

  Serial.println("Thread MessageHandler started.");

  // Initialization: first time the task runs.
  xLastWakeTime = xTaskGetTickCount();

  // Finite State Machine that implements reading 
  // commands from the serial port.
  unsigned char FSM_cmdMsg = 1;

  // Message Handler infnite loop.
  // Periodic task that runs every MESSAGE_HANDLER_TS ms.
  for (;;)
  {
    // Very basic command message processing via Finite State Machine.
    if (Serial.available())
    {
      // Reads one character at a time.
      c = Serial.read();
      cmd_buffer[cmd_buffer_pos] = c;
      cmd_buffer_pos++;

      switch(FSM_cmdMsg)
      {
        case 1:
          if (c == 'e') FSM_cmdMsg++;
          else FSM_cmdMsg = 1;
        break;

        case 2:
          if (c == 'n') FSM_cmdMsg++;
          else FSM_cmdMsg = 1;
        break;

        case 3:
          if (c == 'd') {
            cmd_buffer_pos = cmd_buffer_pos - 3;
            if (cmd_buffer_pos == sizeof(msgCmd_package_t))
              memcpy(&msgCmd_package, cmd_buffer, sizeof(msgCmd_package_t));
          } 
          cmd_buffer_pos = 0;
          FSM_cmdMsg = 1; 

          // Put the command message on the command queue.
          xQueueSend(queueCmd,&msgCmd_package,0);
          FSM_cmdMsg = 1;

        break;
      }
    }
    
    // Try to empty all the data in the queue by sending all available data 
    // down through the serial port.
    while (xQueueReceive(queueData, &msgData_package, 0) == pdTRUE)
    {
      Serial.write((const char *) &msgData_package,sizeof(msgData_package_t));
      Serial.write("end");
    }
    
    // Serial.println("Mensagem recebida.");

    xTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(MESSAGE_HANDLER_TS));
  }

  vTaskDelete(NULL);
}


////////////////////////////////////////////////////////////////////////////
// Control Task:
//  This task takes care of measuring the
//  relevant variables, computing the appropriate
//  control actions, and implementing it at every CONTROL_TASK_TS miliseconds.
void ControlTask(void *pvParameters)
{
  msgData_package_t msgData_package;
  msgCmd_package_t msgCmd_package;

  // Object that represents the control system.
  ControlSystem CtrlSys;
  
  float start_time = 0.0f;
  float t = 0.0f;

  unsigned char ctrl_task_state = 0;

  TickType_t xLastWakeTime;

  // Initialization: first time the task runs.
  xLastWakeTime = xTaskGetTickCount();

  Serial.println("Thread ControlTask started.");
  
  // Initialize the Control System just once.
  CtrlSys.Initialize();

  // Control Task infinite loop.
  // Periodic task that runs every CONTROL_TASK_TS ms.
  for (;;)
  {
    // Process received command messages.
    if (xQueueReceive(queueCmd, &msgCmd_package, 0) == pdTRUE)
    {
      switch (msgCmd_package.cmd)
      {
      case START_CMD:
        ctrl_task_state = 1;

        // Add here other actions that must be performed
        // before starting the experiment.
        start_time = 0.001f*millis();

        CtrlSys.on_start_task();
        
        break;

      case STOP_CMD:
        ctrl_task_state = 0;
        
        // Add here other actions that must be performed
        // before stopping the experiment.
        CtrlSys.on_stop_task();
        
        break;

      case SET_REF_CMD:
        // Programmed reference configuration.
        // CtrlSys.RefSteps.set(msgCmd_package.cmd_val1,msgCmd_package.cmd_val2);
        CtrlSys.RefSteps.set(t,msgCmd_package.cmd_val2);
        break;

      case STORE_REF_CMD:
      break;

      case SET_TS:
        // Cannot change the sample time while the control task 
        // is running control code. Stop, if necessary.
        if (ctrl_task_state == 1) {
          ctrl_task_state = 0;
          CtrlSys.on_stop_task();

          // Delete any remaining data that was put on the data queue.
          xQueueReset(queueData);
        }

        // The actual sample time will be a multiple of CONTROL_TASK_TS.
        // 'cmd_val1' should be the sample time in [ms].
        CtrlSys.set_sample_time(msgCmd_package.cmd_val1);

      break;

      case SET_CTRL_VARIABLE:
        // Cannot change controlled variable while the control task 
        // is running. Stop, if necessary.
        if (ctrl_task_state == 1) {
          ctrl_task_state = 0;
          CtrlSys.on_stop_task();

          // Delete any remaining data that was put on the data queue.
          xQueueReset(queueData);
        }

        // Set the controller strategy
        CtrlSys.set_ctrl_variable((unsigned int) msgCmd_package.cmd_val1);
      break;

      case SET_CTRL_CODE:
        // Cannot change controller strategy while the control task 
        // is running. Stop, if necessary.
        if (ctrl_task_state == 1) {
          ctrl_task_state = 0;
          CtrlSys.on_stop_task();

          // Delete any remaining data that was put on the data queue.
          xQueueReset(queueData);
        }

        // Set the controller strategy
        CtrlSys.set_ctrl_code((unsigned int) msgCmd_package.cmd_val1);
      break;

      case SET_DEADZONE_COMP:
        // Set the inferior (cn) and superior (cp) limits for the dead zone compensation.
        CtrlSys.set_deadzone_comp(msgCmd_package.cmd_val1, msgCmd_package.cmd_val2);
      break;

      case SET_CTRL_SYS_PARAM:
        // Set parameters for a pre-programmed control strategy.
        CtrlSys.set_internal_param((unsigned int) msgCmd_package.cmd_val1, msgCmd_package.cmd_val2);
      break;

      case TEST_COMM:
        // Communication test: GUI in the computer is asking for
        // some confirmation that the ESP32 hardware system is really 
        // connected to the computer.

        // If the control task is running, the reception of this 
        // message will be considered an attempt to reestablish a
        // faulty communication and, therefore, the control task will 
        // stop running and the serial buffer will be flushed:
        if (ctrl_task_state == 1) {
          ctrl_task_state = 0;
          CtrlSys.on_stop_task();

          // Delete any remaining data that was put on the data queue.
          xQueueReset(queueData);
        }

        // Prepare the special data package with very specific values.
        msgData_package.time = -123.0f;  // This negative special value marks this package as a response to a communication test.
        msgData_package.output[0] = msgCmd_package.cmd_val1; // This is used to differentiate from other response packages to earlier requests.  
        msgData_package.output[1] = 0.0f; 
        msgData_package.ctrl_input = CtrlSys.get_ctrl_code_id(); // This informs which control strategy is implemented in the embedded code.
        msgData_package.reference = msgCmd_package.cmd_val2;  // // This is used to differentiate from other response packages to earlier requests.

        // Add the data package to the queue to be read by the MessageHandler task.
        xQueueSend(queueData, &msgData_package, 0);
      break;

      default:
        ctrl_task_state = 0;
        break;
      }
    }

    // If the experiment has started:
    if (ctrl_task_state == 1)
    {
      // Current time since the start of the experiment.
      t = (float) 0.001f * millis() - start_time;
      
      // Data is sent to the message handler task only 
      // if the control algorithm was effectively run.
      // This was introduced to incorporate changes in the
      // sample time without changing the basic periodic scheduling
      // of the control task.
      if (CtrlSys.run(t)) {
        // Prepare the data package.
        msgData_package.time = t;
        msgData_package.output[0] = CtrlSys.get_ym(0);  // Position
        msgData_package.output[1] = CtrlSys.get_ym(1);  // Speed
        msgData_package.ctrl_input = CtrlSys.get_u(0);  // Control Action
        msgData_package.reference = CtrlSys.get_ref(0); // Reference
        
        // Add the data package to the queue to be read by the MessageHandler task.
        xQueueSend(queueData, &msgData_package, 0);
      }

    }

    xTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(CONTROL_TASK_TS));
  }

  vTaskDelete(NULL);
}





///////////////////////////////////////////////////////
// System general initialization.
void setup()
{
  const char *info ="\n\
  *****************************************************\n\
  Department of Electronic Engineering - DELT\n\
  Universidade Federal de Minas Gerais - UFMG\n\
  Date: 09/2025\n\
  Author: Leonardo Torres\n\
  Description: embedded code to run low cost control\n\ 
               lab educational plants.\n\
  *****************************************************\n";

  // Initialize serial port. This is important to 
  // allow error and warnings messages from setting up 
  // sensors and/or actuators to be sent down the serial port
  // if necessary. 
  Serial.begin(115200,SERIAL_8N1);

  Serial.print(info);
  
  // Create queues to implement inter-threads communication.
  queueData = xQueueCreate(QUEUE_DATA_LENGTH, sizeof(msgData_package_t));
  queueCmd = xQueueCreate(QUEUE_CMD_LENGTH,sizeof(msgCmd_package_t));

  if ((queueData == NULL) || (queueCmd == NULL))
    Serial.println("*** Error: creating FIFO queues for threads communication.");
  else
    Serial.println("FIFO queues ok."); 

  // Wait until every character is sent from the buffer to the serial port.
  Serial.flush();

  // Read anything that was lingering in the ESP32 
  // input buffer just in case, before launching
  // the MessageHandler task.
  Serial.read();

  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
      MessageHandler,    /* Task function. */
      "MessageHandler",  /* name of task. */
      10000,             /* Stack size of task */
      NULL,              /* parameter of the task */
      1,                 /* priority of the task */
      &MessageHandler_h, /* Task handle to keep track of created task */
      0);                /* pin task to core 0 */
  delay(200);

  // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      ControlTask,    /* Task function. */
      "ControlTask",  /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      2,              /* priority of the task */
      &ControlTask_h, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  delay(200);
}

void loop() {}
