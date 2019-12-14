#include <iostream>
#include <string>
#include <map>
using namespace std;

int main ()
{

	map<string, map<string, string> > meuMap;
	meuMap["wagner"]["teste"]= "testando";
	cout << meuMap["wagner"]["teste"] << endl;
	return 0;



}
