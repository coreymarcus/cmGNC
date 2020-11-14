#ifndef CMGNC_SRC_EXAMPLE_EXAMPLEINLCUDE_H_
#define CMGNC_SRC_EXAMPLE_EXAMPLEINCLUDE_H_

namespace example {

	class ExampleClass{

	public:
		int pubvar_;
		int PubFun(int input);

	private:
		int privvar_;
		int PrivFun(int input);

	}; //class ExampleClass

} // namespace example

#endif // CMGNC_SRC_EXAMPLE_EXAMPLEHEADER_H_