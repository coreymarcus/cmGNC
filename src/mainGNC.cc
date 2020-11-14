//Standard library headers
#include <iostream>

//libraries from my project
#include "example/example-include.h"

int main() {

	//say hello
	std::cout << "Hello World" << std::endl;
	
	//form an instance of the example class
	example::ExampleClass test;

	//define the public variable
	test.pubvar_ = 6;

	//test our class
	std::cout << test.PubFun(4) << std::endl;
}