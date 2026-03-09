#include <stdio.h>
#include "string_utils.h"
#include <ctype.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <math.h>

// ------------------------------------- Char Conditionals -------------------------------------

bool char_isDigit(char ch) {
	return (ch >= '0' && ch <= '9');
}

bool char_isHex(char ch) {
	return ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F') || (ch >= 'a' && ch <= 'f'));
}

bool char_isAlpha(char ch) {
	return ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'));
}


// ------------------------------------- Char Manipulation -------------------------------------

char char_toUpper(char ch) {
	if(ch >= 'a' && ch <= 'z') {
		ch -= 32;
	}
	return ch;
}

char char_toLower(char ch) {
	if(ch >= 'A' && ch <= 'Z') {
		ch += 32;
	}
	return ch;
}

char getHexDigit(unsigned char num) {
	if(num <= 9) {
		return num + '0';
	} else if(num <= 15) {
		return num + 'A' - 10;
	}
	return '\0';
}

// ------------------------------------ String Conditionals ------------------------------------

unsigned int str_len(char *str) {
	unsigned int i;
	for(i = 0; str[i] != '\0'; i++);
	return i;
}

bool str_isEqual(char *str, char *check) {
	unsigned int i;
	for(i = 0; str[i] != '\0'; i++) {
		if(str[i] != check[i] || check[i] == '\0') {
			return false;
		}
	}
	if(check[i] != '\0') {
		return false;
	}
	return true;
}

bool str_beginsWith(char *str, char *check) {
	unsigned int strlen_str = str_len(str);
	unsigned int strlen_check = str_len(check);
	if(strlen_str < strlen_check) {
		return false;
	}
	for(unsigned int i = 0; i < strlen_check; i++) {
		if(str[i] != check[i]) {
			return false;
		}
	}
	return true;
}

bool str_endsWith(char *str, char *check) {
	unsigned int strlen_str = str_len(str);
	unsigned int strlen_check = str_len(check);
	if(strlen_str < strlen_check) {
		return false;
	}
	for(unsigned int i = 0; i < strlen_check; i++) {
		if(str[i + (strlen_str - strlen_check)] != check[i]) {
			return false;
		}
	}
	return true;
}

bool str_contains(char *str, char *check) {
	unsigned int strlen_str = str_len(str);
	unsigned int strlen_check = str_len(check);
	unsigned int i, j;
	if(strlen_str < strlen_check) {
		return false;
	}
	for(i = 0; i <= (strlen_str - strlen_check); i++) {
		for(j = 0; j < strlen_check; j++) {
			if(str[i+j] != check[j]) {
				break;
			}
		}
		if(j == strlen_check) {
			return true;
		}
	}
	return false;
}

bool str_isInt(char *str) {
	unsigned int i = 0;
	if(str[0] == '+' || str[0] == '-') {
		i++;
	}
	if( str[i] == '\0') {
		return false;
	}
	for(; str[i] != '\0'; i++) {
		if(!char_isDigit(str[i])) {
			return false;
		}
	}
	return true;
}

bool str_isHex(char *str) {
	unsigned int i = 0;
	if(str[0] != '0' && (str[1] != 'x' || str[1] != 'X')) {
		i += 2;
	}
	for(; str[i] != '\0'; i++) {
		if(!char_isHex(str[i])) {
			return false;
		}
	}
	return true;
}

bool str_isFloat(char *str) {
	unsigned int i = 0;
	unsigned char j = 0;
	if(str[0] == '+' || str[0] == '-') {
		i++;
	}
	for(; str[i] != '\0'; i++) {
		if(str[i] == '.') {
			if(++j > 1) {
				return false;
			}
		} else if(!char_isDigit(str[i])) {
			return false;
		}
	}
	return true;
}

bool str_getArgValue(char *str, char *arg, char *val) {
	unsigned int i, j;
	bool flag = false;
	unsigned int strlen_str = str_len(str);
	unsigned int strlen_arg = str_len(arg);

	if(strlen_str < strlen_arg) {
		return false;
	}
	for(i = 0; i <= (strlen_str - strlen_arg); i++) {
		for(j = 0; j < strlen_arg; j++) {
			if(str[i+j] != arg[j]) {
				break;
			}
		}
		if(j == strlen_arg) {
			flag = true;
			break;
		}
	}
	if(!flag) {
		return false;
	}
	i += strlen_arg;
	
	if(str[i] == '\0') {
		val[0] = '\0';
		return true;
	} else if(str[i] != ' ') {
		return false;
	}
	
	i++;
	
	for(j = 0; str[i+j] != ' ' && str[i+j] != '\0'; j++) {
		val[j] = str[i+j];
	}
	val[j] = '\0';
	
	return true;
}

// ------------------------------------ String Manipulation ------------------------------------

void str_cpy(char *src, char *dest) {
	for(; *src != '\0'; src++, dest++) {
		*dest = *src;
	}
	*dest = '\0';
}

void str_toUpper(char *str) {
	for(unsigned int i = 0; str[i] != '\0'; i++) {
		str[i] = char_toUpper(str[i]);
	}
}

void str_toLower(char *str) {
	for(unsigned int i = 0; str[i] != '\0'; i++) {
		str[i] = char_toLower(str[i]);
	}
}

unsigned int str_removeChar(char *str, char ch) {
	unsigned int i, j;
	unsigned int cnt = 0;
	for(i = 0; str[i] != '\0'; i++) {
		if(str[i] == ch) {
			cnt++;
			for(j = i; str[j] != '\0'; j++) {
				str[j] = str[j+1];
			}
		}
	}
	return cnt;
}

void str_concat(char *dest, char *str_a, char *str_b) {
	unsigned int i, j;
	for(i = 0; str_a[i] != '\0'; i++) {
		dest[i] = str_a[i];
	}
	for(j = 0; str_b[j] != '\0'; j++) {
		dest[i+j] = str_b[j];
	}
	dest[i+j] = '\0';
}

signed int str_toInt(char *str) {
	unsigned int i = 0;
	signed int ret = 0, sign = 1;

	if(!str_isInt(str)) {
		return 0;
	}

	if(str[i] == '-') {
		sign = -1;
		i++;
	} else if(str[i] == '+') {
		i++;
	} 
	
	for(; str[i] != '\0'; i++) {
		ret = (ret * 10) + (str[i] - '0');
	}
	
	ret *= sign;

	return ret;
}

unsigned int str_toIntHex(char *str) {
	unsigned int i;
	signed int ret = 0;
	unsigned char offset = 0, digit;

	if(!str_isHex(str)) {
		return 0;
	}

	str_removeChar(str, ' ');
	str_toUpper(str);
	if(str[0] == '0' && str[1] == 'X') {
		offset = 2;
	}
	for(i = offset; str[i] != '\0'; i++) {
		if(str[i] >= '0' && str[i] <= '9') {
			digit = str[i] - '0';
		} else if(str[i] >= 'A' && str[i] <= 'F') {
			digit = str[i] - 'A' + 10;
		} else {
			return 0;
		}
		ret = (ret << 4) | digit;
	}

	return ret;
}

double str_toFloat(char *str) {
	unsigned int i, j;
	double ret = 0;
	unsigned char flag = 0;
	unsigned char isNeg = 0, offset = 0;

	if(!str_isFloat(str)) {
		return 0.0;
	}

	if(str[0] == '-') {
		isNeg = 1;
		offset = 1;
	} else if(str[0] == '+') {
		offset = 1;
	}

	for(i = offset, j = 0; str[i] != '\0'; i++) {
		if (str[i] != '.'){
			ret = (ret * 10) + (str[i] - '0');
			if (flag == 1) {
				j++;
			}
		}
		if (str[i] == '.') {
			if (flag == 1) {
				return 0;
			}
			flag = 1;
		}
	}
	ret = ret / pow(10, j);
	
	if(isNeg) {
		ret = -ret;
	}

	return ret;
}

// ---------------------------------------- To String ----------------------------------------

void intToStr(char *str, signed int num) {
	signed int i;

	if(num < 0) {
		*str++ = '-';
		num = -num;
	}

	for(i = 0; pow(10, i) <= num; i++);
	i--;
	for(; i >= 0; i--) {
		*str++ = ((int)(num / pow(10, i)) % 10) + '0';
	}
	*str = '\0';
}

void hexToStr(char *str, signed int num) {
	signed int i;

	*str++ = '0';
	*str++ = 'x';
	for(i = 0; (1 << (i*4)) <= num; i++);
	if(i % 2) {
	  i++;
	}
	i--;
	for(; i >= 0; i--) {
		*str++ = getHexDigit((num >> i*4) & 0xF);
	}
	*str = '\0';
}

void floatToStr(char *str, double num, unsigned char precision) {
	signed int i;

	if(num < 0) {
		*str++ = '-';
		num = -num;
	}

	for(i = 0; pow(10, i) <= num; i++);
	i--;
	for(; i >= 0; i--) {
		*str++ = ((int)(num / pow(10, i)) % 10)+ '0';
	}
	*str++ = '.';
	for(i = 1; i <= precision; i++) {
		*str++ = (long int)(num * pow(10, i)) % 10 + '0';
	}
	*str = '\0';
}
