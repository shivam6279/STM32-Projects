#ifndef _STRING_UTILS_H
#define _STRING_UTILS_H

#include <stdbool.h>

extern bool char_isDigit(char);
extern bool char_isHex(char);
extern bool char_isAlpha(char);

extern char char_toUpper(char);
extern char char_toLower(char);

extern unsigned int str_len(char*);
extern bool str_isEqual(char*, char*);
extern bool str_beginsWith(char*, char*);
extern bool str_endsWith(char*, char*);
extern bool str_isInt(char*);
extern bool str_isHex(char *str);
extern bool str_isFloat(char *str);
extern bool str_getArgValue(char*, char*, char*);
extern bool str_contains(char*, char*);

extern void str_cpy(char*, char*);
extern void str_toUpper(char*);
extern void str_toLower(char*);
extern unsigned int str_removeChar(char*, char);
extern void str_concat(char*, char*, char*);
extern signed int str_toInt(char *str);
extern unsigned int str_toIntHex();
extern double str_toFloat(char*);

extern void intToStr(char*, signed int);
extern void hexToStr(char*, signed int);
extern void floatToStr(char*, double, unsigned char);

#endif
