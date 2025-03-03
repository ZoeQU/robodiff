MODULE SERVER_L
!***********************************************************
!
! Module:  Module1
!
! Description:
!   <Insert description here>
!
! Author: zoe
!
! Version: 1.0
!
!***********************************************************


!***********************************************************
!
! Procedure main
!
!   This is the entry point of your program
!
!***********************************************************
!////////////////
!GLOBAL VARIABLES
!////////////////

!//Robot configuration
CONST jointtarget jLeftHomePos:=[[0,-130,30,0,40,0],[135,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST jointtarget jLeftHomePos_zoe:=[[-27.19,-68.39,41.76,-69.71,70.44,29.7],[131.96,9E+09,9E+09,9E+09,9E+09,9E+09]];

PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];   
PERS speeddata currentSpeed;
PERS zonedata currentZone;

!// Clock Synchronization
PERS bool startLog:=TRUE;
PERS bool startRob:=TRUE;

!// Mutex between logger and changing the tool and work objects
PERS bool frameMutex:=FALSE;

!//PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
VAR num instructionCode;
VAR num params{10};
VAR num nParams;

LOCAL CONST string ipController:= "192.168.125.1"; !robot default IP
!PERS string ipController:= "192.168.125.108"; !local IP for testing in simulation
!PERS string ipController:= "127.0.0.1"; !local IP for testing in simulation
LOCAL CONST num serverPort:= 5000;

!//Motion of the robot
VAR robtarget cartesianTarget;
VAR jointtarget jointsTarget;
VAR bool moveCompleted; !Set to true after finishing a Move instruction.

!//Buffered move variables
CONST num MAX_BUFFER := 512;
VAR num BUFFER_POS := 0;
VAR robtarget bufferTargets{MAX_BUFFER};
VAR speeddata bufferSpeeds{MAX_BUFFER};

!//External axis position variables
VAR extjoint externalAxis;

!//Circular move buffer
VAR robtarget circPoint;

!//Correct Instruction Execution and possible return values
VAR num ok;
CONST num SERVER_BAD_MSG :=  0;
CONST num SERVER_OK := 1;

!// control gripper
PERS bool bSimulate:=FALSE;
PERS num pos:=10;
PERS num force:=10;
CONST jointtarget jpustThome:=[[-91.9292,-79.7117,-6.79398,121.751,78.8313,-189.909],[62.5319,9E+09,9E+09,9E+09,9E+09,9E+09]];

!// point for test
CONST robtarget p1:=[[-9.58,182.61,198.63],[0.0660107,0.842421,-0.111215,0.523069],[0,0,0,4],[101.964,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget p2:=[[154.43,380.38,198.63],[0.0660106,0.842421,-0.111215,0.523068],[0,0,-1,4],[101.964,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget p4:=[[154.43,380.38,407.86],[0.0660105,0.842421,-0.111216,0.523068],[0,0,-1,4],[80.1356,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget p3:=[[154.43,381.3,329.74],[0.211539,0.818097,-0.0185359,0.534438],[0,0,-1,4],[87.474,9E+09,9E+09,9E+09,9E+09,9E+09]];

VAR robtarget pPushT_Test2:=[[326.95,395.33,236.05],[0.0281063,0.99756,0.0512547,0.0381732],[-1,1,-2,4],[135.001,9E+09,9E+09,9E+09,9E+09,9E+09]];
VAR robtarget pPushT_test3:=[[329.31,394.87,236.00],[0.999974,0.00715588,-0.000172677,0.000263063],[-1,0,1,4],[61.5636,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST jointtarget jpos10:=[[-19.4168,-120.915,53.8464,122.354,78.473,-180.651],[44.8871,9E+09,9E+09,9E+09,9E+09,9E+09]];
PERS tooldata currentTool;
PERS tooldata currentTool2:=[TRUE,[[12.6831,-4.00983,139.604],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
CONST robtarget p10:=[[524.51,214.05,146.11],[0.999975,-0.00707461,-0.000104611,0.000531283],[-1,0,-2,4],[88.9203,9E+09,9E+09,9E+09,9E+09,9E+09]];
PERS tooldata currentTool3:=[TRUE,[[7.95723,2.46905,144.168],[1,0,0,0]],[0.3,[0,0,0.001],[1,0,0,0],0,0,0]];
	
!////////////////
!LOCAL METHODS
!////////////////

!//Go SaftPos
PROC rGoSafetyPos()
        MoveAbsJ jLeftHomePos,v1500,fine,tool0;
    ENDPROC
    
!//Methods for gripper control
!PROC rGripperInit()
!    g_Init\Calibrate,\grip;
!ENDPROC

!PROC rGripperOpen()
!    g_MoveTo pos;
!    WaitTime 0.1;
!ERROR
!    rGripperInit;
!    RETRY;
!ENDPROC

!PROC rGripperClose()
!    IF bSimulate THEN
!        g_MoveTo 8;
!    ELSE
!        g_GripIn\holdForce:=force;
!    ENDIF
!    WaitTime 0.1;
!ENDPROC


!PROC rGripperInit_sim()
!    SetDO Detach_L, 1;
!    WaitTime 0.5;
!    SetDO Detach_L, 0;
!ENDPROC

!//Method to parse the message received from a PC
!// If correct message, loads values on:
!// - instructionCode.
!// - nParams: Number of received parameters.
!// - params{nParams}: Vector of received params.
PROC ParseMsg(string msg)
    !//Local variables
    VAR bool auxOk;
    VAR num ind:=1;
    VAR num newInd;
    VAR num length;
    VAR num indParam:=1;
    VAR string subString;
    VAR bool end := FALSE;
	
    !//Find the end character
    length := StrMatch(msg,1,"#");
    IF length > StrLen(msg) THEN
        !//Corrupt message
        nParams := -1;
    ELSE
        !//Read Instruction code
        newInd := StrMatch(msg,ind," ") + 1;
        subString := StrPart(msg,ind,newInd - ind - 1);
        auxOk:= StrToVal(subString, instructionCode);
        IF auxOk = FALSE THEN
            !//Impossible to read instruction code
            nParams := -1;
        ELSE
            ind := newInd;
            !//Read all instruction parameters (maximum of 8)
            WHILE end = FALSE DO
                newInd := StrMatch(msg,ind," ") + 1;
                IF newInd > length THEN
                    end := TRUE;
                ELSE
                    subString := StrPart(msg,ind,newInd - ind - 1);
                    auxOk := StrToVal(subString, params{indParam});
                    indParam := indParam + 1;
                    ind := newInd;
                ENDIF	   
            ENDWHILE
            nParams:= indParam - 1;
        ENDIF
    ENDIF
ENDPROC


!//Handshake between server and client:
!// - Creates socket.
!// - Waits for incoming TCP connection.
PROC ServerCreateAndConnect(string ip, num port)
    VAR string clientIP;
    VAR string clientPort;
    clientPort := NumToStr(port, 0);
	
    SocketCreate serverSocket;
    SocketBind serverSocket, ip, port;
    SocketListen serverSocket;
    TPWrite "SERVER Left: Server waiting for incoming connections ...";
    WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
        SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP; ! \Time:=WAIT_MAX;
        IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
            TPWrite "SERVER: Problem serving an incoming connection.";
            TPWrite "SERVER Left: Try reconnecting.";
        ENDIF
        !//Wait 0.5 seconds for the next reconnection
        WaitTime 0.5;
    ENDWHILE
    TPWrite "SERVER Left: Connected to IP: " + clientIP + "Port: " + clientPort;
ENDPROC


!//Parameter initialization
!// Loads default values for
!// - Tool.
!// - WorkObject.
!// - Zone.
!// - Speed.
PROC Initialize()
    currentTool := [TRUE,[[7.95723,2.46905,144.168],[1,0,0,0]],[0.3,[0,0,0.001],[1,0,0,0],0,0,0]];
    currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    currentSpeed := [500, 50, 0, 0];
    currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
	
	!Find the current external axis values so they don't move when we start
	jointsTarget := CJointT();
	externalAxis := jointsTarget.extax;
ENDPROC


!////////////////////////
!//SERVER: Main procedure
!////////////////////////
PROC main()
    !//Local variables
    VAR string receivedString;   !//Received string
    VAR string sendString;       !//Reply string
    VAR string addString;        !//String to add to the reply.
    VAR bool connected;          !//Client connected
    VAR bool reconnected;        !//Drop and reconnection happened during serving a command
    VAR robtarget cartesianPose;
    VAR jointtarget jointsPose;
    VAR robtarget robTarget_zoe;
    
    VAR num anglex;
    VAR num angley;
    VAR num anglez;
    VAR pose object;
        
    !VAR num position_L;
    !VAR num speed_L;
    !VAR num torque_L;
    !VAR num exttorque_L;
    VAR robtarget temp_rot;

    !GetJointData 1 \Position:=position_L \Speed:=speed_L \Torque:=torque_L \ExtTorque:=exttorque_L;
	!TPWrite "Current axis1 info: " + NumToStr(position_L,1) + NumToStr(torque_L,1) + NumToStr(exttorque_L,1);
    
    !//Motion configuration
    ConfL \Off;
    SingArea \Wrist;
    moveCompleted:= TRUE;
	
    !//Initialization of WorkObject, Tool, Speed and Zone
    Initialize;
    !MoveAbsJ jpos10\NoEOffs, v1000, z50, tool0;
    MoveAbsJ jpustThome,v1500,fine,currentTool; 
    
    !externalAxis.eax_a := 135.00;
    !TPWrite NumToStr(externalAxis.eax_a,2) + NumToStr(externalAxis.eax_b,2);
    
    !MoveAbsJ jLeftHomePos_zoe,v1500,fine,tool0; 
    !TPWrite "Left: " + taskname_L;
    !TPWrite "Left: " + NumToStr(taskno_L, 0);
    
    !move_test;   ! for speed testing
    !VAR num tcp_speed;
    !tcp_speed := Monitor_speed * 1000; !mm/s
    
    !rGripperInit;
    !rGripperInit_sim;

    !//Socket connection
    connected:=FALSE;
    ServerCreateAndConnect ipController,serverPort;	
    connected:=TRUE;
    
    !pPushT_Test2:= CRobT(\Tool:=currentTool \WObj:=currentWObj);

    !//Server Loop
    WHILE TRUE DO
        !//Initialization of program flow variables
        ok:=SERVER_OK;              !//Correctness of executed instruction.
        reconnected:=FALSE;         !//Has communication dropped after receiving a command?
        addString := "";            

        !//Wait for a command
        SocketReceive clientSocket \Str:=receivedString; !\Time:=WAIT_MAX;
        ParseMsg receivedString;
	    !TPWrite "Left CASE No.: " +  NumToStr(instructionCode, 0);
        !//Execution of the command
        TEST instructionCode
            CASE 0: !Ping
                IF nParams = 0 THEN
                    ok := SERVER_OK;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF

            CASE 1: !Cartesian Move
                !IF nParams = 7 THEN
                    !cartesianTarget :=[[params{1},params{2},params{3}], [params{4},params{5},params{6},params{7}], [0,0,0,0], externalAxis];
                    !TPWrite "Server: x: " + NumToStr(params{1},2) + "y: " + NumToStr(params{2},2) + "z: " + NumToStr(params{3},2);
                    
                IF nParams = 6 THEN
                    cartesianTarget.trans := [params{1},params{2},params{3}];
                    temp_rot.rot := OrientZYX(params{4}, params{5}, params{6});
                    
                    TPWrite "Server: x: " + NumToStr(cartesianTarget.trans.x,2) + "y: " + NumToStr(cartesianTarget.trans.y,2) +  "z: " + NumToStr(cartesianTarget.trans.z,2);
                    !TPWrite "Server: y: " + NumToStr(cartesianTarget.trans.y,2);
                    !TPWrite "Server: z: " + NumToStr(cartesianTarget.trans.z,2);
                    
                    !TPWrite "Server: q1: " + NumToStr(cartesianTarget.rot.q1,2);
                    !TPWrite "Server: q2: " + NumToStr(cartesianTarget.rot.q2,2);
                    !TPWrite "Server: q3: " + NumToStr(cartesianTarget.rot.q3,2);
                    !TPWrite "Server: q4: " + NumToStr(cartesianTarget.rot.q4,2);
                    
                    !TPWrite "Server: robconf 1: " + NumToStr(cartesianTarget.robconf.cf1,0);
                    !TPWrite "Server: robconf 2: " + NumToStr(cartesianTarget.robconf.cf4,0);
                    !TPWrite "Server: robconf 3: " + NumToStr(cartesianTarget.robconf.cf6,0);
                    !TPWrite "Server: robconf 4: " + NumToStr(cartesianTarget.robconf.cfx,0);
                    
                    cartesianTarget :=[[params{1},params{2},params{3}], [temp_rot.rot.q1,temp_rot.rot.q2,temp_rot.rot.q3,temp_rot.rot.q4], [0,0,0,4], externalAxis];
                    !TPWrite NumToStr(externalAxis.eax_a,2);
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveL cartesianTarget, currentSpeed, currentZone, currentTool;
                    moveCompleted := TRUE;
                    WaitTime 0.01;

                    !pPushT_Test2 := Offs(pPushT_Test2,params{1},params{2},params{3});
                    !ok := SERVER_OK;
                    !MoveL pPushT_Test2, currentSpeed, currentZone, currentTool \WObj:=currentWobj;
                    !moveCompleted := TRUE;
                    !WaitTime 0.01;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF	
				
            CASE 2: !Joint Move
                IF nParams = 6 THEN
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}], externalAxis];
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                    moveCompleted := TRUE;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 3: !Get Cartesian Coordinates (with current tool and workobject)
                IF nParams = 0 THEN
                    cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);		
                    addString := NumToStr(cartesianPose.trans.x,2) + " ";
                    addString := addString + NumToStr(cartesianPose.trans.y,2) + " ";
                    addString := addString + NumToStr(cartesianPose.trans.z,2) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q1,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q2,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q3,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q4,3); !End of string	
                    ok := SERVER_OK;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 4: !Get Joint Coordinates
                IF nParams = 0 THEN
                    jointsPose := CJointT();
                    
                    addString := NumToStr(jointsPose.robax.rax_1,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_2,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_3,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_4,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_5,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_6,2) + " "; 
                    addString := addString + NumToStr(jointsPose.extax.eax_a,2); !End of string
                    ok := SERVER_OK;
                    
                    !TPWrite addString;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                
			CASE 5: !Get external axis positions
                IF nParams = 0 THEN
                    jointsPose := CJointT();
                    addString := StrPart(NumToStr(jointsTarget.extax.eax_a, 2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_b,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_c,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_d,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_e,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_f,2),1,8); !End of string
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF	
		
            CASE 6: !Set Tool
                IF nParams = 7 THEN
        		   WHILE (frameMutex) DO
        		        WaitTime .01; !// If the frame is being used by logger, wait here
        		   ENDWHILE
            		    frameMutex:= TRUE;
                        currentTool.tframe.trans.x:=params{1};
                        currentTool.tframe.trans.y:=params{2};
                        currentTool.tframe.trans.z:=params{3};
                        currentTool.tframe.rot.q1:=params{4};
                        currentTool.tframe.rot.q2:=params{5};
                        currentTool.tframe.rot.q3:=params{6};
                        currentTool.tframe.rot.q4:=params{7};
                        ok := SERVER_OK;
		            frameMutex:= FALSE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 7: !Set Work Object
                IF nParams = 7 THEN
                    currentWobj.oframe.trans.x:=params{1};
                    currentWobj.oframe.trans.y:=params{2};
                    currentWobj.oframe.trans.z:=params{3};
                    currentWobj.oframe.rot.q1:=params{4};
                    currentWobj.oframe.rot.q2:=params{5};
                    currentWobj.oframe.rot.q3:=params{6};
                    currentWobj.oframe.rot.q4:=params{7};
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 8: !Set Speed of the Robot
                IF nParams = 4 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    currentSpeed.v_leax:=params{3};
                    currentSpeed.v_reax:=params{4};
                    ok := SERVER_OK;
                ELSEIF nParams = 2 THEN
					currentSpeed.v_tcp:=params{1};
					currentSpeed.v_ori:=params{2};
					ok := SERVER_OK;
				ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 9: !Set zone data
                IF nParams = 4 THEN
                    IF params{1}=1 THEN
                        currentZone.finep := TRUE;
                        currentZone.pzone_tcp := 0.0;
                        currentZone.pzone_ori := 0.0;
                        currentZone.zone_ori := 0.0;
                    ELSE
                        currentZone.finep := FALSE;
                        currentZone.pzone_tcp := params{2};
                        currentZone.pzone_ori := params{3};
                        currentZone.zone_ori := params{4};
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                
            CASE 10: !Go Home Position
                IF nParams = 0 THEN
                    rGoHome;
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                
            !CASE 20: !Open gripper 
            !    IF nParams = 0 THEN
            !!        rGripperOpen;
            !        SetDO Detach_L, 1;
            !        ok := SERVER_OK;
            !    ELSE
            !        ok:=SERVER_BAD_MSG;
            !    ENDIF
                    
            !CASE 21: ! Close gripper
            !    IF nParams = 0 THEN
            !!        rGripperClose;
            !        SetDO Detach_L, 0;
            !        ok := SERVER_OK;
            !    ELSE
            !        ok:=SERVER_BAD_MSG;
            !    ENDIF
                    

            CASE 30: !Add Cartesian Coordinates to buffer
                IF nParams = 7 THEN
                    cartesianTarget :=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    IF BUFFER_POS < MAX_BUFFER THEN
                        BUFFER_POS := BUFFER_POS + 1;
                        bufferTargets{BUFFER_POS} := cartesianTarget;
                        bufferSpeeds{BUFFER_POS} := currentSpeed;
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 31: !Clear Cartesian Buffer
                IF nParams = 0 THEN
                    BUFFER_POS := 0;	
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 32: !Get Buffer Size)
                IF nParams = 0 THEN
                    addString := NumToStr(BUFFER_POS,2);
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 33: !Execute moves in cartesianBuffer as linear moves
                IF nParams = 0 THEN
                    FOR i FROM 1 TO (BUFFER_POS) DO 
                        !SetDO Detach_L,1;
                        !SetDO Detach_L,0;
                        MoveL bufferTargets{i}, bufferSpeeds{i}, currentZone, currentTool;
                    ENDFOR			
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 34: !External Axis move
                IF nParams = 6 THEN
                    externalAxis :=[params{1},params{2},params{3},params{4},params{5},params{6}];
                    jointsTarget := CJointT();
                    jointsTarget.extax := externalAxis;
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                    moveCompleted := TRUE;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 35: !Specify circPoint for circular move, and then wait on toPoint
                IF nParams = 7 THEN
                    circPoint :=[[params{1},params{2},params{3}],
                                [params{4},params{5},params{6},params{7}],
                                [0,0,0,0],
                                externalAxis];
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 36: !specify toPoint, and use circPoint specified previously
                IF nParams = 7 THEN
                    cartesianTarget :=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    MoveC circPoint, cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
				
            CASE 98: !returns current robot info: serial number, robotware version, and robot type
                IF nParams = 0 THEN
                    addString := GetSysInfo(\SerialNo) + "*";
                    addString := addString + GetSysInfo(\SWVersion) + "*";
                    addString := addString + GetSysInfo(\RobotType);
                    ok := SERVER_OK;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF
			
            CASE 99: !Close Connection
                IF nParams = 0 THEN
                    TPWrite "SERVER: Client has closed connection.";
                    connected := FALSE;
                    !//Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;

                    !Reinitiate the server
                    ServerCreateAndConnect ipController,serverPort;
                    connected := TRUE;
                    reconnected := TRUE;
                    ok := SERVER_OK;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF
            DEFAULT:
                TPWrite "SERVER: Illegal instruction code";
                ok := SERVER_BAD_MSG;
        ENDTEST
		
        !Compose the acknowledge string to send back to the client
        IF connected = TRUE THEN
            IF reconnected = FALSE THEN
			    IF SocketGetStatus(clientSocket) = SOCKET_CONNECTED THEN
				    sendString := NumToStr(instructionCode,0);
                    sendString := sendString + " " + NumToStr(ok,0);
                    sendString := sendString + " " + addString;
                    SocketSend clientSocket \Str:=sendString;
			    ENDIF
            ENDIF
        ENDIF
    ENDWHILE

ERROR (LONG_JMP_ALL_ERR)
    TPWrite "SERVER: ------";
    TPWrite "SERVER: Error Handler:" + NumtoStr(ERRNO,0);
    TEST ERRNO
        CASE ERR_SOCK_CLOSED:
            TPWrite "SERVER: Lost connection to the client.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort;
            reconnected:= FALSE;
            connected:= TRUE;
            RETRY; 
        DEFAULT:
            TPWrite "SERVER: Unknown error.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort;
            reconnected:= FALSE;
            connected:= TRUE;
            RETRY;
    ENDTEST
ENDPROC

PROC rGoHome()
    MoveAbsJ jLeftHomePos,v1500,fine,tool0; 
ENDPROC

PROC move_test()
    MoveAbsJ jLeftHomePos, v100, fine, tool0;
    MoveJ p1, v200, z30, tool0;
    WaitTime 0.5;
    MoveJ p2, v100,z30,tool0; 
    WaitTime 0.5;
    MoveJ p3, v200, z30, tool0;
    WaitTime 0.5;
    MoveJ p4, v100,z30,tool0;

ENDPROC


ENDMODULE