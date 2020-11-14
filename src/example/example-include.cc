#include "example-include.h"

namespace example{

	// default constructor
	ExampleClass::ExampleClass(){}

	int ExampleClass::PubFun(int input){
		return input;
	}

	int ExampleClass::PrivFun(int input){
		return input;
	}


} //namespace example