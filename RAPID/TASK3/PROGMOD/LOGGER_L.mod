MODULE LOGGER_L

!////////////////
!GLOBAL VARIABLES
!////////////////
!PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
PERS string ipController:="192.168.125.1";
!PERS num loggerPort:= 5002;
LOCAL CONST num loggerPort:=5002;

!Robot configuration	
PERS tooldata currentTool;    
PERS wobjdata currentWobj;
VAR speeddata currentSpeed;
VAR zonedata currentZone;

!//Logger sampling rate
!PERS num loggerWaitTime:= 0.01;  !Recommended for real controller
!PERS num loggerWaitTime:= 0.1;    !Recommended for virtual controller

PERS num loggerWaitTime:= 0.0065 ; ! set to ~128Hz

PROC ServerCreateAndConnect(string ip, num port)
	VAR string clientIP;
	
	SocketCreate serverSocket;
	SocketBind serverSocket, ip, port;
	SocketListen serverSocket;
	TPWrite "LOGGER: Logger waiting for incomming connections ...";
	WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
		SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP;    ! \Time:=WAIT_MAX;
		IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
			TPWrite "LOGGER: Problem serving an incomming connection.";
			TPWrite "LOGGER: Try reconnecting.";
		ENDIF
		 !Wait 0.5 seconds for the next reconnection
		 WaitTime 0.5;
	ENDWHILE
	TPWrite "LOGGER: Connected to IP " + clientIP;
ENDPROC

PROC main()
	VAR string data;
    VAR string temp_data;
	VAR robtarget position;
	VAR jointtarget joints;
    VAR string sendString;
	VAR bool connected;
    VAR byte data_buffer;
    VAR num rx;
    VAR num ry;
    VAR num rz;
    
	VAR string date;
	VAR string time;
	VAR clock timer;
    
    VAR num position_L;
    VAR num speed_L;
    VAR num torque_L;
    VAR num exttorque_L;
        
    date:= CDate();
	time:= CTime();
    ClkStart timer;
    
    !tasknameLLogger := GetTaskName(\MecTaskNo:=tasknoLLogger);
    !TPWrite "Logger: " + NumToStr(tasknoLLogger, 0);
    
	connected:=FALSE;
	ServerCreateAndConnect ipController,loggerPort;
	connected:=TRUE;

	WHILE TRUE DO
		!Cartesian Coordinates
		position := CRobT(\TaskName:="T_ROB_L" \Tool:=tool0 \WObj:=wobj0);
        !Joint Coordinates
		joints := CJointT(\TaskName:="T_ROB_L");
        
		data := "#0|";
		!data := data + date + " " + time + " ";
		!data := data + NumToStr(ClkRead(timer),2) + " ";
		
        data := data + NumToStr(position.trans.x,1) + ",";
		data := data + NumToStr(position.trans.y,1) + ",";
		data := data + NumToStr(position.trans.z,1) + ",";
	        
        !For now, drop the rotation infomation.
        rx := EulerZYX(\X, position.rot); 
        data := data + NumToStr(rx,1) + ",";
        ry := EulerZYX(\Y, position.rot); 
        data := data + NumToStr(ry,1) + ",";
        rz := EulerZYX(\Z, position.rot); 
        data := data + NumToStr(rz,1) + "|";    !End of string	
        
        !IF connected = TRUE THEN
	    !	SocketSend clientSocket \Str:=data;
	    !ENDIF
        !WaitTime loggerWaitTime;
	
		data := data + "#1|";
		!data := data + date + " " + time + " ";
		!data := data + NumToStr(ClkRead(timer),2) + " ";
		data := data + NumToStr(joints.robax.rax_1,1) + ",";
		data := data + NumToStr(joints.robax.rax_2,1) + ",";
		data := data + NumToStr(joints.robax.rax_3,1) + ",";
		data := data + NumToStr(joints.robax.rax_4,1) + ",";
		data := data + NumToStr(joints.robax.rax_5,1) + ",";
		data := data + NumToStr(joints.robax.rax_6,1) + ","; !End of string
		data := data + NumToStr(joints.extax.eax_a,1) + "@";  !End of string

        IF connected = TRUE THEN
			SocketSend clientSocket \Str:=data;
		ENDIF
        
		WaitTime loggerWaitTime;
        
	ENDWHILE
	ERROR
	IF ERRNO=ERR_SOCK_CLOSED THEN
		TPWrite "LOGGER: Client has closed connection.";
	ELSE
		TPWrite "LOGGER: Connection lost: Unknown problem.";
	ENDIF
	connected:=FALSE;
	!Closing the server
	SocketClose clientSocket;
	SocketClose serverSocket;
	!Reinitiate the server
	ServerCreateAndConnect ipController,loggerPort;
	connected:= TRUE;
	RETRY;
ENDPROC

ENDMODULE