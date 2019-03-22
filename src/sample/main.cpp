#include <fstream>
#include <iostream>

int main() {
	
	// 以写模式打开文件
	std::ofstream outfile;
	outfile.open("afile.dat");
	
	// 向文件写入用户输入的数据
	outfile << 12345 << std::endl;
	
	// 关闭打开的文件
	outfile.close();
	
	return 0;
}
