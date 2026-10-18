#ifndef _DIAGS_H_
#define _DIAGS_H_

// Diags menu. Lines come from console.c when a port is in diags mode (typed "diags").
void DiagsMenu();
void DiagsRun(char *cmd);
void DiagsEspTask();	// commands from the ESP32 app (!boot)
void DiagsPoll();	// 'btn mon' / 'can mon' output

#endif
