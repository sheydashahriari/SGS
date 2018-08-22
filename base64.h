#ifndef  _BASE64_HPP_
#define _BASE64_HPP_

const char  BASE64CODEBANK[] ="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";
inline void  BASE64 ( char in1,char in2 , char in3 ,char  & out1,char & out2,char & out3,char & out4 , char _count )
{
	if ( _count  == 3 )
	{
		out1=BASE64CODEBANK[(in1>>2)];
		out2=BASE64CODEBANK[((in1<<4)|(in2>>4))&63];
		out3=BASE64CODEBANK[((in2<<2)|(in3>>6))&63];
		out4=BASE64CODEBANK[(in3&63)];
	}
	else if ( _count  == 2 )
	{
		out1=BASE64CODEBANK[(in1>>2)];
		out2=BASE64CODEBANK[((in1<<4)|(in2>>4))&63];
		out3=BASE64CODEBANK[((in2<<2))&63];
		out4=BASE64CODEBANK[64];
	}
	else 
	{
		out1=BASE64CODEBANK[(in1>>2)];
		out2=BASE64CODEBANK[(in1<<4)&63];
		out3=BASE64CODEBANK[64];
		out4=BASE64CODEBANK[64];
	}
}






#endif