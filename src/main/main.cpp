#include <ros/ros.h>

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <string>
using namespace std;

using std::cin;
using std::cout;
using std::endl;
using std::string;

int main(int argc, char** argv)
{
	string name_str("main.exe");
	ROS_INFO ( "[%s():%d:]Application(%s) running !", __FUNCTION__, __LINE__, name_str.c_str());

	ros::init ( argc, argv, "main_testxx" );
	
	char *p_name = (char *)name_str.c_str();

	printf("1 Application = %s\n", p_name);

	std::cout<<"2 Application"<<std::endl;
	cout<<"3 Application"<<endl;
	
	string name("4 hello!");
	std::cout<<name<<std::endl;

	string str("533 leon!");
	printf("%s\n", str.c_str());
	
	int ret = getchar();
	printf("%c\n", ret);
	
	return 0;
}

