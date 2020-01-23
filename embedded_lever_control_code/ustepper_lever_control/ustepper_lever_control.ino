#include <uStepper.h>

const int MAXACCELERATION = 1500;         //Max acceleration = 1500 Steps/s^2
const int MAXVELOCITY = 550;           //Max velocity = 1100 steps/s
uStepper stepper(MAXACCELERATION, MAXVELOCITY);

typedef enum
{
  UNUSED        = 0,
  INIT          = 1,
  READY 		    = 2,
  TRAIL_RUNNING = 3,
  TRAIL_ENDED   = 4
  
}operation_t;

typedef enum
{
  DC            = 0,
  ENDPOINT      = 1,
  MID           = 2,
  RANDOM        = 3
}reset_t;

typedef enum
{
  NA		      = 0,
  setGoal     = 1,
  getGoal 	  = 2,
  getAbsPos   = 3,
  getRelPos   = 4,
  getReward   = 5,
  resetEnvABS = 6,
  resetEnvRAN = 7,
  envReady 	  = 8,
  startTrail  = 9,
  resetEnvMID = 10
  
}request_t;

int encoder_pos = 0;  

const int encoder_MIN_POS = -500;
const int encoder_MAX_POS = 500;
const int encoder_MIDDLE_POS = (encoder_MAX_POS + encoder_MIN_POS)/2;

int const BUFFER_SIZE = 64;
int const GOAL_BYTE_SIZE = 2;
int serial_buffer[BUFFER_SIZE];

//flags
int encoder_init_done = 0;

int motor_disabled = 0;

int encoder_state_change = 0;

int env_initialized = 0;

operation_t OPERATION_MODE = INIT;
request_t PREVIOUS_REQUEST = NA;

//PARAMETERS YOU MAY CHANGE

const int DEBUG_VERBOSE = 0;
const int STEP_INC = 10;
const int SLACK = 1;

const int MAX_REWARD = 100;
const int REG_REWARD = 0;

int goal_pos = 0;

/////////////////////////////////////////////////

void init_stepper_motor()
{
  stepper.setup();
  stepper.softStop(SOFT);
}

void init_serial_com()
{
  Serial.begin(115200);
  if(DEBUG_VERBOSE)
  {
    Serial.println("Serial initialized!");
  }
}

void take_step(bool dir, int steps)
{
  if(DEBUG_VERBOSE)
  {
    Serial.println("Taking step!");
  }
  if((encoder_MIN_POS < encoder_pos) and (encoder_pos < encoder_MAX_POS))
  {
  	stepper.moveSteps(steps, dir, SOFT);
  }
  else
  {
    if(DEBUG_VERBOSE)
    {
      Serial.println("!!!! MOTOR AT POS LIMIT !!!!"); 
    }
  }
}

void take_n_steps(bool dir, int n)
{
  int total_steps = STEP_INC * n;
  take_step(dir, total_steps);
}

void invert_motor_state()
{
  if(motor_disabled)
  {
    stepper.hardStop(HARD);
    motor_disabled = 0;
  }
  else
  {
    stepper.hardStop(SOFT);
    motor_disabled = 1;
  }
}

void read_encoder(void)
{
  encoder_pos = int(10 * stepper.encoder.getAngleMoved());
}

void print_encoder_data()
{
  Serial.println("The encoder counts: ");
  Serial.print(encoder_pos);
  Serial.println(" steps.");
}

void motor_move_to_pos(int target_pos)
{
  int initial_pos = get_encoder_pos();

  if (DEBUG_VERBOSE)
  {
    Serial.println();
    Serial.println("------------------------------");
    Serial.print("Moving to position: ");
    Serial.println(target_pos);
    Serial.print("From position: ");
    Serial.println(initial_pos);
    Serial.println("------------------------------");
  }

  if((target_pos == (initial_pos + SLACK)) || (target_pos == (initial_pos - SLACK)))
  {
    //This block is empty on purpose.
  }
  else if(target_pos < initial_pos)
  {
    stepper.runContinous(CW);
    while(target_pos < get_encoder_pos())
    {
      read_encoder();
      if(DEBUG_VERBOSE)
      {
        Serial.print("Current position: ");
        Serial.println(get_encoder_pos());
      }
    }
  }
  else if(target_pos > initial_pos)
  {
    stepper.runContinous(CCW);
    while(target_pos > get_encoder_pos())
    {
      read_encoder();
      if(DEBUG_VERBOSE)
      {
        Serial.print("Current position: ");
        Serial.println(get_encoder_pos());
      }
    }
  }
  stepper.hardStop(SOFT);
}

void reset_env(reset_t endpoint)
{
  if(DEBUG_VERBOSE)
  {
    Serial.println("RESETTING ENV -- RESETTING ENV -- RESETTING ENV");
    Serial.println();
    Serial.println();
  } 

  stepper.hardStop(HARD);

  motor_move_to_pos(encoder_MIDDLE_POS);

  int target_pos = 0;
  
  switch(endpoint)
  {
    case DC:
    {
      target_pos = get_encoder_pos();
      break;
    }
    case ENDPOINT:
    {
      target_pos = encoder_MAX_POS;
      break;
    }
    case MID:
    {
      target_pos = encoder_MIDDLE_POS;
      break;
    }
    case RANDOM:
    {
      target_pos = random(encoder_MIN_POS, encoder_MAX_POS);
      break;
    }
  }

  motor_move_to_pos(target_pos);

  env_initialized = 1;
  set_operation_mode(READY);
}

request_t int_to_request(int request)
{
  request_t result = NA;
  switch(request)
  {
  	case 0:
  	{
  	  result = NA;
  	  break;
  	}
    case 1:
    {
      result = setGoal;
      break;
    }
    case 2:
    {
      result = getGoal;
      break;
    }
    case 3:
    {
      result = getAbsPos;
      break;
    }
    case 4:
    {
      result = getRelPos;
      break;
    }
    case 5:
    {
      result = getReward;
      break;
    }
    case 6:
    {
      result = resetEnvABS;
      break;
    }
    case 7:
    {
      result = resetEnvRAN;
      break;
    }
    case 8:
    {
      result = envReady;
      break;
    }
    case 9:
    {
 	    result = startTrail;
      break;
    }
    case 10:
    {
      result = resetEnvMID;
      break;
    }
  }
  return result;
}

void read_serial_data()
{
  if (Serial.available())
  {
    int command = Serial.parseInt();

    if(DEBUG_VERBOSE)
    {
      Serial.print("PARSED INT: ");
      Serial.println(command);
    }

    if(command)
    {
      request_t new_req = int_to_request(command);
      handle_serial_request(new_req);
    }
    else
    {
      if(DEBUG_VERBOSE)
      {
        Serial.println("Nothing new on serial!");
      }
    }
  }
}

void write_serial_data(int data)
{
  Serial.println(data);
}

void handle_serial_request(request_t s_req)
{
  switch(s_req)
  {
    case setGoal:
    {
        while(Serial.available() < GOAL_BYTE_SIZE);
      	int new_goal = Serial.parseInt();
      	
      	if (DEBUG_VERBOSE)
      	{
      		Serial.println();
      		Serial.println("--------");
      		Serial.print("New goal parsed is: ");
      		Serial.println(new_goal);
      		Serial.println("--------");
      		Serial.println();
      	}

      	set_goal_pos(new_goal);
      	break;
    }
    case getGoal:
    {
      	int goalState = get_goal_pos();
      	write_serial_data(goalState);
      	break;
    }
    case getAbsPos:
    {
      	int absPos = get_abs_pos();
      	write_serial_data(absPos);
      	break;
    }
    case getRelPos:
    {
      	int relPos = get_relative_pos();
      	write_serial_data(relPos);
      	break;
    }
    case getReward:
    {
      	int reward = get_reward();
      	write_serial_data(reward);
      	break;
    }
    case resetEnvABS:
    {
        reset_t endpoint = ENDPOINT;
      	reset_env(endpoint);
      	break;  
    }
    case resetEnvRAN:
    {
        reset_t endpoint = RANDOM;
      	reset_env(endpoint);
      	break;
    }
    case envReady:
    {
      	int envState = env_isReady();
      	write_serial_data(envState);
      	break;
    }
    case startTrail:
    {
        if(env_initialized)
        {
          env_initialized = 0;
    	    set_operation_mode(TRAIL_RUNNING);
          write_serial_data(1);
        }
        else
        {
          write_serial_data(0);
        }
        break;
    }
    case resetEnvMID:
    {
      reset_t endpoint = MID;
      reset_env(endpoint);
      break;
    }
  }

  PREVIOUS_REQUEST = s_req;
}


void set_operation_mode(operation_t new_mode)
{
  OPERATION_MODE = new_mode;
}

void set_goal_pos(int new_goal)
{
  if((encoder_MIN_POS <= new_goal) and (new_goal <= encoder_MAX_POS))
  {
    goal_pos = new_goal;   
  }
  else
  {
    char error_msg[30];
    sprintf(error_msg, "GOAL %d OUT OF RANGE", new_goal);
    Serial.println(error_msg);
  }
}

int get_goal_pos()
{
  return goal_pos;
}

int env_isReady()
{
  return env_initialized;
}

int get_reward()
{
  //NBNB implementere spicy/non-sparse kostfunksjon her?
  int relative_distance = get_relative_pos();

  if(relative_distance <= SLACK)
  {
    set_operation_mode(TRAIL_ENDED);
    return MAX_REWARD;
  }
  else
  {
    return REG_REWARD;
  }
}

int get_encoder_pos()
{
  return encoder_pos;
}

int get_relative_pos()
{
  int relative_dist = 0;
  int current_pos = get_encoder_pos();
  int goal = get_goal_pos();

  int current_pos_ABS = abs(current_pos);
  int goal_ABS = abs(goal);

  int highest = 0;
  int lowest = 0;

  if(current_pos_ABS <= goal_ABS)
  {
    highest = goal_ABS;
    lowest = current_pos_ABS;
  }
  else
  {
    highest = current_pos_ABS;
    lowest = goal_ABS;
  }

  if(((goal <= 0) && (current_pos <= 0)) || ((goal >= 0) && (current_pos >= 0)))
  {
    relative_dist = highest - lowest;
  }

  else if((goal <= 0) && (current_pos >= 0))
  {
    relative_dist = current_pos - goal; 
  }
  else if((goal >= 0) && (current_pos <= 0))
  {
    relative_dist = goal - current_pos;
  }

  if(goal <= current_pos)
  {
    relative_dist = -relative_dist;
  }
  
  return relative_dist;
}

int get_abs_pos()
{
  return encoder_pos;
}

void setup() 
{
  // put your setup code here, to run once:
 init_stepper_motor();
 init_serial_com();
}

void loop() 
{
  // put your main code here, to run repeatedly:

  read_encoder();
  
  read_serial_data();

  // if(DEBUG_VERBOSE)
  // {
  //   Serial.print("Encoder is reading: ");
  //   Serial.println(get_encoder_pos());
  //   Serial.println("");
  //   Serial.print("OPERATING IN MODE: ");
  //   Serial.println(OPERATION_MODE);
  // }

  switch(OPERATION_MODE)
  {
    case READY:
    {
      break;
    }

    case TRAIL_RUNNING:
    {
      if (!motor_disabled)
      {
        invert_motor_state();
      }
      break;
    }

    case TRAIL_ENDED:
    {
      if (motor_disabled)
      {
        invert_motor_state();
      }
      break;
    }
  }  
}
