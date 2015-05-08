#include "picopter.h"

int main(int argc, char *argv[]) {
	picopter::FlightBoard fb;
	char buf[BUFSIZ];
	int val = 0;
	
	while (true) {
		printf("Gimbal: ");
		fgets(buf, BUFSIZ, stdin);
		val = atoi(buf);
		fb.SetGimbal(val);
	}
	
}
