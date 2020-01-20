const int STEP_PIN = 14;
const int DIR_PIN = 12;
const int EN_PIN = 5;
const int BUTTON_PIN = 4;
const int CHA_A_PIN = 15;
const int CHA_B_PIN = 13;

const int ENCODER_STATES[4] = {0, 1, 2, 3};

const int INIT_RANDOM = 0;
const int INIT_ENDPOINT = 1;

const int CLOCKWISE = LOW;
const int COUNTER_CLOCKWISE = HIGH;

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
  ERROR		    = 0,
  setGoal     = 1,
  getGoal 	  = 2,
  getAbsPos   = 3,
  getRelPos   = 4,
  getReward   = 5,
  resetEnvABS = 6,
  resetEnvRAN = 7,
  envReady 	  = 8,
  startTrail  = 9
  
}request_t;

int cha_A_state = 0;
int cha_B_state = 0;
int encoder_state_index = 0;
int encoder_previous_state_index = 0;

int encoder_pos = 0;
int encoder_previous_dir = 0;  

const int encoder_MIN_POS = 0;
const int encoder_MAX_POS = 20;
const int encoder_MIDDLE_POS = encoder_MAX_POS/2;

int const BUFFER_SIZE = 64;
int const GOAL_BYTE_SIZE = 2;
int serial_buffer[BUFFER_SIZE];


//flags
int encoder_init_done = 0;

int button_state = LOW;
int motor_disabled = LOW;

int encoder_state_change = 0;
int motor_running = 0;

int env_initialized = 0;
int lever_init_complete = 0;

operation_t OPERATION_MODE = INIT;

//PARAMETERS YOU MAY CHANGE

const int DEBUG_VERBOSE = 1;
const int STEP_INC = 10;
const int SLACK = 1;

const int MAX_REWARD = 100;
const int REG_REWARD = 0;

int goal_pos = 0;

/////////////////////////////////////////////////

void take_step(int dir)
{
  if(DEBUG_VERBOSE)
  {
    Serial.println("Taking step!");
  }
  if((encoder_MIN_POS < encoder_pos) and (encoder_pos < encoder_MAX_POS))
  {
  	digitalWrite(DIR_PIN, dir);
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delay(1);  
  }
  else
  {
    if(DEBUG_VERBOSE)
    {
      Serial.println("!!!! MOTOR AT POS LIMIT !!!!"); 
    }
  }
}

void take_n_steps(int dir, int n)
{
  motor_running = 1;
  for(int i = 0; i < n; i++)
  {
    take_step(dir);
  }
  motor_running = 0;
}

void init_stepper_driver()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(EN_PIN, LOW);
}

void init_pushbutton()
{
  pinMode(BUTTON_PIN, INPUT);
}

void invert_motor_state()
{
  if(motor_disabled)
  {
    motor_disabled = LOW;
  }
  else
  {
    motor_disabled = HIGH;
  }
  digitalWrite(EN_PIN, motor_disabled);
}

void disable_motor()
{
  motor_disabled = HIGH;
  digitalWrite(EN_PIN, motor_disabled); 
}

void enable_motor()
{
    motor_disabled = LOW;
    digitalWrite(EN_PIN, motor_disabled);
}

void init_magnetic_encoder()
{
  pinMode(CHA_A_PIN, INPUT);
  pinMode(CHA_B_PIN, INPUT);
  read_encoder();
  encoder_pos = encoder_MIDDLE_POS;
  encoder_init_done = 1;
}

void init_serial_com()
{
  Serial.begin(9600);
  Serial.println("Serial initialized!");
}

void read_encoder(void)
{
  if(!encoder_init_done)
  {
  	cha_A_state = digitalRead(CHA_A_PIN);
    cha_B_state = digitalRead(CHA_B_PIN);
  }

  int temp_A_state = digitalRead(CHA_A_PIN);
  int temp_B_state = digitalRead(CHA_B_PIN);
  
  if(temp_A_state != cha_A_state)
  {
    cha_A_state = temp_A_state;
    encoder_state_change = 1;
  }
  else if(temp_B_state != cha_B_state)
  {
    cha_B_state = temp_B_state;
    encoder_state_change = 1;
  }
  else
  {
    if(DEBUG_VERBOSE)
    {
      Serial.println("Nothing new from Encoder!");  
    }
  }
}

void update_encoder_pos_dir()
{
  if(((encoder_previous_state_index == 0) and (encoder_state_index == 3)) 
   or (ENCODER_STATES[encoder_previous_state_index - 1] == encoder_state_index))
  {
    encoder_previous_dir = COUNTER_CLOCKWISE;
    encoder_pos--;
  }
  else if(((encoder_previous_state_index == 3) and (encoder_state_index == 0)) 
        or (ENCODER_STATES[encoder_previous_state_index + 1] == encoder_state_index))
  {
    encoder_previous_dir = CLOCKWISE;
    encoder_pos++;
  }
  else
  {
    if(DEBUG_VERBOSE)
    {
      Serial.println("!!!!!!!!!!!!");
      Serial.println("Something is wrong with the encoding. Perhaps it updates at no new change?");  
    }
  }
}

void update_encoder_data()
{
  encoder_previous_state_index = encoder_state_index;
  if(cha_A_state)
  {
    if(cha_B_state)
    {
      encoder_state_index = 2;
    }
    else
    {
      encoder_state_index = 1;
    }
  }
  else
  {
    if(cha_B_state)
    {
      encoder_state_index = 3; 
    }
    else
    {
      encoder_state_index = 0;
    }
  }
  
  update_encoder_pos_dir();
  
  encoder_state_change = 0;
}

void print_encoder_internal_states()
{
  Serial.println("The encoder has the following internal states!");
  Serial.print("CHANNEL A: ");
  Serial.println(cha_A_state);
  Serial.print("CHANNEL B: ");
  Serial.println(cha_B_state);
}

void print_encoder_data()
{
  Serial.println("The encoder counts: ");
  Serial.print(encoder_pos);
  Serial.println(" steps.");

  Serial.print("and latest direction: ");
  Serial.println(encoder_previous_dir);
}



void init_endpoint_lever_position()
{
  //motor step in a direction, until encoder stops changing.
  int target_pos = encoder_MAX_POS;

  int done = 0;

  while(!done)
  {
    take_n_steps(CLOCKWISE, STEP_INC);
    
    int current_pos = get_encoder_pos();
    
    if(target_pos <= (current_pos + SLACK))
    {
      done = true;
    }
  }
  lever_init_complete = true;
  set_operation_mode(READY);
}

void init_random_lever_position()
{
  int target_pos = random(encoder_MIN_POS, encoder_MAX_POS);
  int initial_pos = get_encoder_pos();

  if(target_pos < initial_pos)
  {
    while(target_pos < get_encoder_pos())
    {
       take_n_steps(COUNTER_CLOCKWISE, STEP_INC);
    }
  }
  else
  {
    while(target_pos > get_encoder_pos())
    {
      take_n_steps(CLOCKWISE, STEP_INC);
    }
  }
  lever_init_complete = true;
  set_operation_mode(READY);
}

void motor_move_to_pos(int target_pos)
{
  int current_pos = get_encoder_pos();
  
  while(target_pos > (current_pos - SLACK))
  {
    take_step(COUNTER_CLOCKWISE);
  }
  while(target_pos < (current_pos + SLACK))
  {
    take_step(CLOCKWISE);
  }
}

void reset_env(int endpoint)
{
  if(DEBUG_VERBOSE)
  {
    Serial.println("RESETTING ENV -- RESETTING ENV -- RESETTING ENV");
    Serial.println();
    Serial.println();
  } 

  enable_motor();

  motor_move_to_pos(encoder_MIDDLE_POS);

  if(endpoint)
  {
    init_endpoint_lever_position();
  }
  else
  {
    init_random_lever_position();
  }

  while((motor_running) and (!lever_init_complete))
  {
    continue;
  }
  set_operation_mode(READY);
  env_initialized = 1;
}

request_t int_to_request(int request)
{
  request_t result = ERROR;
  switch(request)
  {
  	case 0:
  	{
  	  result = ERROR;
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
  Serial.print(data);
  Serial.print('\n');
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
      	reset_env(INIT_ENDPOINT);
      	break;  
    }
    case resetEnvRAN:
    {
      	reset_env(INIT_RANDOM);
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
      }
      break;
    }
  }
}


void set_operation_mode(operation_t new_mode)
{
  OPERATION_MODE = new_mode;
}

void set_goal_pos(int new_goal)
{
  goal_pos = new_goal;
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

int get_encoder_dir()
{
  return encoder_previous_dir;
}

int get_relative_pos()
{
  int relative_dist = 0;
  int current_pos = get_encoder_pos();
  int goal = get_goal_pos();

  if(goal <= current_pos)
  {
    relative_dist = current_pos - goal;
  }
  else
  {
  	relative_dist = goal - current_pos;
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
 init_serial_com();
 init_pushbutton();
 init_stepper_driver();
 init_magnetic_encoder();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if(digitalRead(BUTTON_PIN))
  {
    if(DEBUG_VERBOSE)
    {
      Serial.print("Current motor state is: ");
      Serial.println(motor_disabled);
      Serial.println("Inverting motor state! "); 
    }
    invert_motor_state();
    delay(1000);
  }

  read_encoder();
  
  if(encoder_state_change)
  {
    update_encoder_data();
  }
  
  read_serial_data();

  if(DEBUG_VERBOSE)
  {
    print_encoder_internal_states();
    print_encoder_data();
    Serial.println("");
    Serial.print("OPERATING IN MODE: ");
    Serial.println(OPERATION_MODE);
  }

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
        disable_motor();
      }
      break;
    }

    case TRAIL_ENDED:
    {
      if (motor_disabled)
      {
        enable_motor();
      }
      break;
    }
  }  
}
