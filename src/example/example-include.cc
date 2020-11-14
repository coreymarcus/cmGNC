#include "example-include.h"

namespace example{

	// default constructor
	ExampleClass::ExampleClass(){}

	int ExampleClass::PubFun(int input){
		
		//set the private variable to the input
		privvar_ = input;

		//call the private function
		return PrivFun();
	}

	int ExampleClass::PrivFun(){
		return privvar_ + pubvar_;
	}


} //namespace example