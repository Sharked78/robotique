project:
	g++ carto.cpp -lAria -I/usr/local/Aria/include -L/usr/local/Aria/lib

simple:
	g++ simpleMotionCommands.cpp -lAria -I/usr/local/Aria/include -L/usr/local/Aria/lib

laser:
	g++ lasers.cpp -lAria -I/usr/local/Aria/include -L/usr/local/Aria/lib

goto:
	g++ gotoActionExample.cpp -lAria -I/usr/local/Aria/include -L/usr/local/Aria/lib

ae:
	g++ actionExample.cpp -lAria -I/usr/local/Aria/include -L/usr/local/Aria/lib
