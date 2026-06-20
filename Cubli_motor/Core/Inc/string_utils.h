#ifndef _STRING_UTILS_H
#define _STRING_UTILS_H

#include <stdbool.h>

bool char_isDigit(char);
bool char_isHex(char);
bool char_isAlpha(char);

char char_toUpper(char);
char char_toLower(char);

unsigned int str_len(char*);
bool str_isEqual(char*, char*);
bool str_beginsWith(char*, char*);
bool str_endsWith(char*, char*);
bool str_isInt(char*);
bool str_isHex(char *str);
bool str_isFloat(char *str);
bool str_getArgValueN(char*, char*, char*, unsigned int);
// Wrapper: auto-passes the destination buffer size so the copy can't overflow.
// val MUST be an array (not a pointer), otherwise sizeof gives the pointer size.
#define str_getArgValue(str, arg, val) str_getArgValueN((str), (arg), (val), sizeof(val))
bool str_contains(char*, char*);

void str_cpy(char*, char*);
void str_toUpper(char*);
void str_toLower(char*);
unsigned int str_removeChar(char*, char);
void str_concat(char*, char*, char*);
signed int str_toInt(char *str);
unsigned int str_toIntHex();
float str_toFloat(char*);

void intToStr(char*, signed int);
void hexToStr(char*, signed int);
void floatToStr(char*, double, unsigned char);

#endif
