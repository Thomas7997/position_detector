#include "lcd.h"

void Write(char f)
{
	PORT_CHAR->ODR &= ~(1 << DB4);
	PORT_CHAR->ODR &= ~(1 << DB5);
	PORT_CHAR->ODR &= ~(1 << DB6);
	PORT_CHAR->ODR &= ~(1 << DB7);

	if ((f & 0b10000000) != 0)
		PORT_CHAR->ODR |= (1 << DB7);
	if ((f & 0b01000000) != 0)
		PORT_CHAR->ODR |= (1 << DB6);
	if ((f & 0b00100000) != 0)
		PORT_CHAR->ODR |= (1 << DB5);
	if ((f & 0b00010000) != 0)
		PORT_CHAR->ODR |= (1 << DB4);
}

void toggle_e()
{
	Port_E->ODR |= (1 << E);

	LL_mDelay(50);

	Port_E->ODR &= ~(1 << E);
}
void D_set_E_Toggle(char f)
{
	Write(f);
	toggle_e();
}

void WriteFunction(char f)
{
	char high_nibble;
	char low_nibble;

	Port_RS->ODR &= ~(1 << rs);

	D_set_E_Toggle(f);

	high_nibble = (f & 0b11110000) >> 4;
	low_nibble = f & 0b00001111;
	int swap = high_nibble | (low_nibble << 4);

	D_set_E_Toggle(swap);

	LL_mDelay(50);
}

void WriteChar(char c)
{
	Port_RS->ODR |= (1 << rs);
	D_set_E_Toggle(c);

	char high_nibble = (c & 0b11110000) >> 4;
	char low_nibble = c & 0b00001111;
	int swap = high_nibble | (low_nibble << 4);

	D_set_E_Toggle(swap);
	LL_mDelay(50);
}

void WriteChaine(char ch[])
{
	int i = 0;
	while (ch[i] != '\0')
	{
		WriteChar(ch[i]);
		i++;
	}
}

void lcdinit4()
{
	LL_mDelay(1000000);

	PORT_CHAR->MODER &= ~(0b11 << (2 * DB4));
	PORT_CHAR->MODER |= (0b01 << (2 * DB4));
	PORT_CHAR->MODER &= ~(0b11 << (2 * DB5));
	PORT_CHAR->MODER |= (0b01 << (2 * DB5));
	PORT_CHAR->MODER &= ~(0b11 << (2 * DB6));
	PORT_CHAR->MODER |= (0b01 << (2 * DB6));
	PORT_CHAR->MODER &= ~(0b11 << (2 * DB7));
	PORT_CHAR->MODER |= (0b01 << (2 * DB7));

	Port_E->MODER &= ~(0b11 << (2 * E));
	Port_E->MODER |= (0b01 << (2 * E));

	Port_RS->MODER &= ~(0b11 << (2 * rs));
	Port_RS->MODER |= (0b01 << (2 * rs));

	LL_mDelay(20000);

	Write(0x30);
	toggle_e();
	LL_mDelay(5000);

	Write(0x30);
	toggle_e();
	LL_mDelay(200);

	Write(0x30);
	toggle_e();
	LL_mDelay(200);

	Write(0x20);
	toggle_e();
	LL_mDelay(200);

	WriteFunction(0x28);

	// DISPLAY OFF
	WriteFunction(0x8);

	// DISPLAY CLEAR
	WriteFunction(1);
	LL_mDelay(1600); // delay=1.6ms

	// ENTRY MODE SET
	WriteFunction(0x6);

	// DISPLAY ON
	WriteFunction(0xC);
}

void Affichage_LCD(char *ligne1, char *ligne2)
{
	// LCD <- 01, display clear
	WriteFunction(0x80);
	WriteFunction(1);
	LL_mDelay(1600);

	// Set new cursor
	WriteFunction(0x80);
	WriteChaine(ligne1);

	// Set new cursor
	WriteFunction(0xC0);
	WriteChaine(ligne2);
}
