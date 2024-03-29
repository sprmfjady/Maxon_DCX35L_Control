/*
* MCU_Init.c
*
* Created: 2019-04-20 ���� 3:10:08
* Author: Administrator
*/

#include "mcu_init.h"


//////////////////////////////////////////////////////////////////
//InitIO()
//Initialize Input & Output of Port
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitIO(){

	//TO DO
	DDRA = 0xFF;
	DDRC = 0xFF;
	DDRD = 0x08;
	DDRB = 0x67;	//PWM & SPI
	DDRE = 0x1A;

	PORTA = 0x00;
	PORTB = 0x07;
}



//////////////////////////////////////////////////////////////////
//InitExtInt()
//Initialize External Interrupt
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitExtInt(){
	
	//TO DO
	DDRD = 0x00;
	
	EICRA = INT1_FALLING | INT0_FALLING;
	
	EIMSK = INT1_ENABLE | INT0_ENABLE;
}



//////////////////////////////////////////////////////////////////
//InitTimer0()
//Initialize Timer0
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitTimer0(){
	
	//TO DO
	TCCR0 = 0x04;
	TIMSK = 0x01;
}



//////////////////////////////////////////////////////////////////
//InitTimer1()
//Initialize Timer1
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitTimer1(){
	
	//TO DO
	TCCR1A = 0b11100010;
	TCCR1B = 0b00010001;
		
	ICR1 = 399;
	OCR1C = 0;
	
	OCR1A = 0;		//1 L
	OCR1B = 0;		//2 L
	
	TCNT1 = 0;

}



//////////////////////////////////////////////////////////////////
//InitTimer2()
//Initialize Timer2
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitTimer2(){
	
	//TO DO
}



//////////////////////////////////////////////////////////////////
//InitTimer3()
//Initialize Timer3
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitTimer3(){
	
	//TO DO
	//TCCR3A = 0b00000000;
	//TCCR3B = 0b00000011;

	//ETIMSK = 0x04;

	TCCR3A = 0b10110010;
	TCCR3B = 0b00010001;
		
	ICR3 = 399;
	OCR3C = 0;
	
	OCR3A = 0;		//1 L
	OCR3B = 0;		//2 L
		
	TCNT3 = 0;
}



//////////////////////////////////////////////////////////////////
//InitADC()
//InitADC
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitADC(){
	
	//TO DO
	ADMUX = 0x40;
	ADCSRA = 0x86;
}



//////////////////////////////////////////////////////////////////
//GetADC()
//GetADC
// Input : adc chanel
// Output : ADC Result
//////////////////////////////////////////////////////////////////
int GetADC(char ch){
	
	//TO DO
	ADMUX = (ADMUX & 0xf0) + ch;
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}



//////////////////////////////////////////////////////////////////
//InitUart0()
//InitUart0
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitUart0(){
	
	//TO DO
	//Uart
	UCSR0A = 0x00;
	UCSR0B = 0x98;
	UCSR0C = 0x06;
	
	UBRR0L = 103;
}



//////////////////////////////////////////////////////////////////
//InitUart1()
//InitUart1
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitUart1(){
	
	//TO DO
	DDRD = (DDRD & 0xF3) | 0x08;
	
	UCSR1A = 0x00;
	UCSR1B = USART_RECV_ENABLE | USART_TRANS_ENABLE;
	UCSR1C = USART_CHAR_SIZE_8BIT;
	
	UBRR1L = USART_115200BPS;
}


//////////////////////////////////////////////////////////////////
//InitSPI()
//InitSPI
// Input : None
// Output : None
//////////////////////////////////////////////////////////////////
void InitSPI(){
	
	SPCR = 0x50;
	SPSR = 0x01;
}



//////////////////////////////////////////////////////////////////
//TransUart0()
//TransUart0
// Input : Transmit data
// Output : None
//////////////////////////////////////////////////////////////////
void TransUart0(unsigned char data){
	
	//TO DO
	while(!(UCSR0A & 0x20));
	UDR0 = data;
}



//////////////////////////////////////////////////////////////////
//TransUart1()
//TransUart1
// Input : Transmit data
// Output : None
//////////////////////////////////////////////////////////////////
void TransUart1(unsigned char data){
	
	//TO DO
	while(!(UCSR1A & 0x20));
	UDR1 = data;
}



//////////////////////////////////////////////////////////////////
//RecvUart0()
//RecvUart0
// Input : None
// Output : Recved Data
//////////////////////////////////////////////////////////////////
unsigned char RecvUart0(){
	
	//TO DO
	
	return UDR0;
}



//////////////////////////////////////////////////////////////////
//RecvUart1()
//RecvUart1
// Input : None
// Output : Recved Data
//////////////////////////////////////////////////////////////////
unsigned char RecvUart1(){
	
	//TO DO
	
	return UDR1;
}



//////////////////////////////////////////////////////////////////
//TransNumUart0()
//TransNumUart0
// Input : Number data
// Output : None
//////////////////////////////////////////////////////////////////
void TransNumUart0(int num){
	
	//TO DO
	if(num < 0){
		TransUart0('-');
		num *= -1;
	}
	
	TransUart0( ((num%10000000) / 1000000) + 48);
	TransUart0( ((num%1000000) / 100000) + 48);
	TransUart0( ((num%100000) / 10000) + 48);
	TransUart0( ((num%10000) / 1000) + 48);
	TransUart0( ((num%1000) / 100) + 48);
	TransUart0( ((num%100) / 10) + 48);
	TransUart0( num%10 + 48 );
}



//////////////////////////////////////////////////////////////////
//SendShortUART0()
//SendShortUART0
// Input : Number data
// Output : None
//////////////////////////////////////////////////////////////////
void SendShortUART0(int16_t num){
	
	if(num < 0){
		TransUart0('-');
		num *= -1;
	}

	TransUart0( ((num%100000) / 10000) + 48);
	TransUart0( ((num%10000) / 1000) + 48);
	TransUart0( ((num%1000) / 100) + 48);
	TransUart0( ((num%100) / 10) + 48);
	TransUart0( num%10 + 48 );
}



//////////////////////////////////////////////////////////////////
//TransNumUart1()
//TransNumUart1
// Input : Number data
// Output : None
//////////////////////////////////////////////////////////////////
void TransNumUart1(int num){
	
	//TO DO
	if(num < 0){
		TransUart1('-');
		num *= -1;
	}
	
	TransUart1( ((num%10000000) / 1000000) + 48);
	TransUart1( ((num%1000000) / 100000) + 48);
	TransUart1( ((num%100000) / 10000) + 48);
	TransUart1( ((num%10000) / 1000) + 48);
	TransUart1( ((num%1000) / 100) + 48);
	TransUart1( ((num%100) / 10) + 48);
	TransUart1( num%10 + 48 );
}



//////////////////////////////////////////////////////////////////
//SendShortUART1()
//SendShortUART1
// Input : Number data
// Output : None
//////////////////////////////////////////////////////////////////
void SendShortUART1(int16_t num){
	
	if(num < 0){
		TransUart1('-');
		num *= -1;
	}

	TransUart1( ((num%100000) / 10000) + 48);
	TransUart1( ((num%10000) / 1000) + 48);
	TransUart1( ((num%1000) / 100) + 48);
	TransUart1( ((num%100) / 10) + 48);
	TransUart1( num%10 + 48 );
}


//////////////////////////////////////////////////////////////////
//SPI_MasterSend()
//SPI_MasterSend
// Input : data
// Output : None
//////////////////////////////////////////////////////////////////
void SPI_MasterSend(unsigned char data){
	
	SPDR = data;
	while (!(SPSR & 0x80));
	data = SPDR;
}



//////////////////////////////////////////////////////////////////
//SPI_MasterSend()
//SPI_MasterSend
// Input : None
// Output : data
//////////////////////////////////////////////////////////////////
unsigned char SPI_MasterRecv(void)
{
	SPDR = 0x00;
	while (!(SPSR & 0x80));
	return SPDR;
}