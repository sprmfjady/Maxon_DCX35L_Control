/*
 * GccApplication1.c
 *
 * Created: 2019-08-28 오후 12:13:16
 * Author : CDSL
 */ 

#include "mcu_init.h"
#include "dataType.h"

#define F_CPU 16000000UL

#include <math.h> // PI 사용 위해 설정

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

volatile int32_t g_Cnt, g_preCnt;

volatile double g_Pdes = 0., g_Ppre;
volatile double g_Pcur, g_Pvcur;
volatile double g_Perr;

volatile double g_Vcur, g_Vpre;
volatile double g_Vdes = 10;
volatile double g_Verr;
volatile double g_Vlimit = 1.;    // 목표 속도 제한  speed limit -Vlimit ~ +Vlimit

volatile double g_Ccur;
volatile double g_Cdes = 0.05;
volatile double g_Cerr;
volatile double g_Cerr_sum;
volatile double g_Climit = 1.;

volatile double g_ADC;
volatile int g_SendFlag = 0;
volatile int g_Direction;

volatile int cur_control = 0;
volatile double g_vel_control;
volatile double g_pos_control;
volatile unsigned char g_TimerCnt;

volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;

volatile double dt_vel = 0.005;
volatile double Ke=0.0683;	  //Back_EMF constant
volatile double Kt=0.0683;	  //Torque constant
volatile double Kg=0.012345   //Gear ratio

// 변수 추가
volatile double g_Pderr;
volatile double g_Verr_sum;

// gain tuning을 통해 제어 게인 설정
volatile double Kp_p = 15.53;
volatile double Kd_p = 0.1;    

volatile double Kp_s = 2.32;
volatile double Ki_s = 75.54;

volatile double Kp_c = 0.8269;
volatile double Ki_c = 2211.7;

//Distuebance Observer variable
  volatile double J_eq=0.00001; //Jm = 0.01;             // inertia of Geard Mortor    // J_eq = 9.9501*e-06
  volatile double b_eq=0.0026;							// nominal friction of Geard Mortor 
  volatile double tau = 0.001;                   //  time constant of Q-filter          
  volatile double w_cur_DOB = 0;                      //  w = predicted input+predicted disturb
  volatile double w_pre_DOB = 0;
  volatile double y_cur_DOB = 0;                       // output (omega) 
  volatile double y_pre_DOB = 0;
  volatile double u_cur_DOB = 0;                       // input (current)
  volatile double u_filtered_cur_DOB =0;																
  volatile double u_filtered_pre_DOB = 0;
  volatile double d_DOB = 0;                              // predicted disturbance    d = w-u
  volatile double Ktg = Kt*Kg;
  volatile double g_vel_control_pre=0;



// 모터에 PWN값 지정해주는 함수
void SetDutyCW(double v){
	
	while(TCNT1 == 0);

	int ocr = v * (200. / 24.) + 200;
	
	if(ocr > OCR_MAX)	ocr = OCR_MAX;
	else if(ocr < OCR_MIN)	ocr = OCR_MIN;  //OCR : 10~390
	//OCR1A = OCR1B = ocr;
	
	OCR1A = OCR3B = ocr + 8;		//1 H
	OCR1B = OCR3A = ocr - 8;		//1 L
}

void InitLS7366(){
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR0 | WR_REG);
	SPI_MasterSend(X4_QUAD | FREE_RUN | DISABLE_INDEX | SYNCHRONOUS_INDEX |FILTER_CDF_1);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR1 | WR_REG);
	SPI_MasterSend(FOUR_BYTE_COUNT_MODE | ENABLE_COUNTING);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_CNTR | CLR_REG);
	PORTB = 0x01;
}

//ADC를 받아오는 함수 : 전류센서 값
int getADC(char ch){

	ADMUX = (ADMUX & 0xf0) + ch;
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}

// USART0번 인터럽트 >> UART0번을 통해 MFC로부터 값이 들어오면 발동
// 여기서부터 ATMega128의 역할 시작
ISR(USART0_RX_vect){

	g_buf[g_BufWriteCnt++] = UDR0; // 값이 들어오면 값이 들어오는 대로 버퍼에 값들 저장
}

// 타이머 인터럽트 (제어 주기 설정)
ISR(TIMER0_OVF_vect){
			
	TCNT0 = 256 - 125;  // 131 -> 제어주기:0.5ms
	//TCNT3 = 65536 - 125;
	//Read LS7366
	int32_t cnt;
	
	PORTC = 0x01;
	
	g_ADC = getADC(0); // ADC 0번 값을 읽어 옴 : ADC 0번 (PF0)은 전류센서 연결
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | LOAD_REG);  // 0010 1000 | 1100 0000 = 1110 1000
	PORTB = 0x01;
			
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | RD_REG);
	cnt = SPI_MasterRecv();		cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();
	PORTB = 0x01;
	g_Cnt = -cnt;
	
	PORTC = 0x03;
	
	g_Pcur = (g_Cnt / (4096. * 81.)) * 2 * M_PI;  // 엔코더로 받은 현재 샤프트 위치 정보
	
	
	//제어 코드(위치, 속도, 전류 제어기)//
	
	// 위치제어기
	if ((g_TimerCnt % 100) == 0) {
				
		if (g_Pdes < 0) {
			g_Pdes + 2*M_PI; // 음수 값 입력시 해당되는 양수 값으로 출력
		}
		
		//위치 제어의 결과 값으로 속도 값이 도출됨
		g_Perr = g_Pdes - g_Pcur;  // 목표 위치 - 현재 위치 = position error
		g_Pderr = g_Perr - g_Ppre; // 현재 에러 - 이전 에러 = position error 미분
		g_pos_control = (double) g_Perr * Kp_p +  g_Pderr* Kd_p; // PD 제어기
		
		if (g_pos_control > 642.65/81.) {
			g_pos_control = 642.65/81.;
		}
		
		else if (g_pos_control < -642.65/81.) {
			g_pos_control = -642.65/81.;
		}
		
		g_Ppre = g_Perr; // 현재 위치 에러 값을 이전 위치 에러 값으로 저장
		g_TimerCnt = 0;
	}
		
	// 속도제어기
	if ((g_TimerCnt % 10) == 0) {
		
		// 속도 값 Saturation
		if (g_pos_control > g_Vlimit) {
			g_pos_control = g_Vlimit;
		}
		else if (g_pos_control < -g_Vlimit) {
			g_pos_control = -g_Vlimit;
		}
		
		// 속도 제어의 결과 값으로 전류 값이 도출됨
		g_Vcur = (g_Pcur - g_Pvcur) / 0.005;                       // 현재속도
		g_Verr = g_pos_control - g_Vcur;                           // velocity error
		g_vel_control = g_Verr * Kp_s + g_Verr_sum * Ki_s * 0.005; // PI 제어기
		g_Verr_sum += g_Verr;                                                          // 속도 값에 대한 에러 값을 계속 누적
		
		// 속도 error sum saturation
		if (g_Verr_sum > 30.) {
			g_Verr_sum = 30.;
		}
		
		else if (g_Verr_sum < -30.) {
			g_Verr_sum= -30.;
		}



	    /////////////////////////////////////////////////////
        ///////////////////////DOB///////////////////////////
        y_cur_DOB = g_Vcur;
        u_cur_DOB = g_vel_control_pre;
        
        w_cur_DOB = (J_eq*(y_cur_DOB-y_pre_DOB)+tau*Ktg*w_pre_DOB+b_eq*y_pre_DOB)/(tau*Ktg+dt_vel*Ktg); 
        u_filtered_cur_DOB = (dt_vel*u_cur_DOB+tau*u_filtered_pre_DOB)/(tau+dt_vel);  
       
        d_DOB = w_cur_DOB - u_filtered_cur_DOB;
       
        w_pre_DOB = w_cur_DOB;
        y_pre_DOB = y_cur_DOB;
        u_filtered_pre_DOB = u_filtered_cur_DOB;
        
        g_vel_control -= d_DOB;

	    g_vel_control_pre = g_vel_control;

		
		// 최대 허용 전류 값에 대한 saturation & anti-windup
		if (g_vel_control > g_Climit) {
			g_Verr_sum -= (g_vel_control - g_Climit) * (1. / Kp_s); // anti windup gain : 1/Kps
			g_vel_control = g_Climit;
		}
		
		else if (g_vel_control < -g_Climit) {
			g_Verr_sum -= (g_vel_control + g_Climit) * (1. / Kp_s); // anti windup gain : 1/Kps
			g_vel_control = -g_Climit;
		}
		
		g_Pvcur = g_Pcur; // 현재 엔코더 값을 이전 엔코더 값으로 저장
	}
	
	g_Cdes = g_vel_control; // 속도 제어 값을 목표 전류 값으로 다시 저장

	// 전류 값 Saturation
	if (g_Cdes > g_Climit) {
		g_Cdes = g_Climit;
	}
	else if (g_Cdes < -g_Climit) {
		g_Cdes = -g_Climit;
	}
		
	g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.5) * 10.);           // 전류 센서를 통해 받은 전류 값(ADC)을 통해 현재 전류 값 계산
	g_Cerr = g_Cdes - g_Ccur;				                   // (목표 전류 - 현재 전류) = current error
	cur_control = g_Cerr * Kp_c + g_Cerr_sum * Ki_c * 0.0005;  // PI 제어기:  200hz이므로 0.0005초에 한번 이므로 적분부에 0.005를 곱해줌
	cur_control += g_Vcur * 0.0683;				               // 역기전력 전향 보상
	g_Cerr_sum += g_Cerr;		                               // current error sum(I-term)
	

	

	//Anti-Windup
	if (cur_control >= 24) {						    // 최대 출력 24V
		g_Cerr_sum -= (cur_control - 24.) * 1. /  Kp_c;	// Anti-Windup 계수 1/3kp
		cur_control = 24;
	}
	else if (cur_control < -24) {
		g_Cerr_sum -= (cur_control + 24.) * 1. / Kp_c;	// Anti-Windup 계수 1/3kp
		cur_control = -24;
	}
	
	SetDutyCW(cur_control);  //목표 전압 전송

	/////////////////////////////////////////
	g_TimerCnt++;
	g_SendFlag++;
}




int main(void){
	
	Packet_t packet;
	packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFE; // 패킷을 보낼 때 맨 앞 header 데이터 설정
	
	InitIO();
	
	//Uart
	InitUart0(); // atmega128에서 MFC로 serial 통신을 위한 USART 레지스터 설정 함수
	
	//SPI
	InitSPI();
	
	//Timer
	InitTimer0();
	InitTimer1();
	InitTimer3();


	TCNT1 = TCNT3 = 0;
	SetDutyCW(0.);
	
	//ADC
	InitADC();
	
	//LS7366
	InitLS7366();
	
	TCNT0 = 256 - 125;
	//TCNT3 = 65536 - 125;
	sei();

	unsigned char check = 0;
	
    while (1) {
		 //패킷 통신 데이터 해석 코드
		for(;g_BufReadCnt != g_BufWriteCnt; g_BufReadCnt++){
			// 반복문 선언: 버퍼로 받은 패킷 데이터 다 읽을때 까지 for문 반복
			// SendShortUART1(g_PacketMode); TransUart1(32); TransUart1(13);
			// Packet 모드에 따라 switch문 발동
			
			switch(g_PacketMode){
				
			case 0:
				
				if (g_buf[g_BufReadCnt] == 0xFF) {
					checkSize++;
					if (checkSize == 4) {
						g_PacketMode = 1;
					}
				}
				else {
					checkSize = 0;
				}
				break;
				
			case 1:

				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				
				if (checkSize == 8) {
					if(g_PacketBuffer.data.id == g_ID){

						g_PacketMode = 2;
					}
					else{
						g_PacketMode = 0;
						checkSize = 0;
					}
				}

				break;
			
			case 2:
				
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				check += g_buf[g_BufReadCnt];
				
				if (checkSize == g_PacketBuffer.data.size) {

					if(check == g_PacketBuffer.data.check){

						switch(g_PacketBuffer.data.mode){

							case 2:
							g_Pdes = g_PacketBuffer.data.pos / 1000.;
							g_Vlimit = g_PacketBuffer.data.velo / 1000.;
							g_Climit = g_PacketBuffer.data.cur / 1000.;
							break;
							}
					}
					
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;
				}
				else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)) {
					//TransUart0('f');
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;
				}
			}
		}


		
		if(g_SendFlag > 19){
			g_SendFlag = 0;
				
			packet.data.id = g_ID;
			packet.data.size = sizeof(Packet_data_t);
			packet.data.mode = 3;
			packet.data.check = 0;
			
			packet.data.pos = g_Pcur * 1000; 
			packet.data.velo = g_Vcur * 1000;
			packet.data.cur = g_Ccur * 1000;
			
			for (int i = 8; i < sizeof(Packet_t); i++)
			packet.data.check += packet.buffer[i];
			
			for(int i=0; i<packet.data.size; i++){
				TransUart0(packet.buffer[i]);
			}
		}
	}
		
}

