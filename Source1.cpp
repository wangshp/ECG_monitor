#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <stdlib.h>
#include <cmath>
#include <bitset>

using namespace std;
int main() {
	//char* argve[];
	char* argv[] = { "", "config.txt", "signal.txt" };


	ifstream traces;
	ofstream tracesout;
	ofstream tracesout2;
	string outname, outname2;
	outname = string(argv[2]) + ".txt";
	outname2 = string(outname) + ".txt";

	traces.open(argv[2]);
	tracesout.open(outname.c_str());
	tracesout2.open(outname2.c_str());
	string line;
	string accesstype;  // the Read/Write access type from the memory trace;
	string xaddr;       // the address from the memory trace store in hex;
	string charactor;
	string charactor_value;
	string notify;
	string data_value1;
	string data_value2;
	string accesstype2;
	unsigned int addr;  // the address from the memory trace store in unsigned int;        
	bitset<32> accessaddr; // the address from the memory trace store in the bitset;

	unsigned long value_interger;
	

	if (traces.is_open() && tracesout.is_open() && tracesout2.is_open()) {
		while (getline(traces, line)) {   // read mem access file and access Cache

			istringstream iss(line);
			if (!(iss >> accesstype >> xaddr >> charactor >> charactor_value >> notify>> data_value1 >> data_value2)) { break; }
			stringstream saddr(data_value2);
			saddr >> std::hex >> addr;
			accessaddr = bitset<32>(addr);
			value_interger = accessaddr.to_ulong();
			//cout << data_value1 << " " << data_value2;
			//while (1)
			//	;
			char *time = &accesstype[0u];
			if (accesstype != accesstype2)
			{
				tracesout << value_interger << endl;  // Output hit/miss results for L1 and L2 to the output file;
				tracesout2 << time[6] << time[7] << time[8] << time[9] << time[10] << time[11] << endl;
				accesstype2 = accesstype;
			}
		}
		traces.close();
		tracesout.close();
		tracesout2.close();
	}
	else cout << "Unable to open trace or traceout file ";


	return 0;
}