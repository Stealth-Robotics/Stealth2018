#ifndef JORSUTILS_H
#define JORSUTILS_H
 
long doubleToLong(double x);
void putU32IntoU8Array(char* data, int location, long value);
void putU16IntoU8Array(char* data, int location, int value);
int getU16FrombyteArray(char* data, int location);

#endif

