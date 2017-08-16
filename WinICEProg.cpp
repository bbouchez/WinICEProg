

/*
 *  iceprogduino -- simple programming tool for FTDI-based Lattice iCE programmers
 *  and Olimexino-32U4 based programmers for iCE40HX1K-EVB and iCEHX8K-EVB
 *
 *  Copyright (C) 2015  Clifford Wolf <clifford@clifford.at>
 *  Olimexino-32U4 edition by Chris B. <chris@protonic.co.uk> @ Olimex Ltd. <c> 2016
 *  Project ported to Visual Studio 2012 for Windows by Benoit Bouchez <benoit@imodular-synth.com> (c) 2017
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  Relevant Documents:
 *  -------------------
 *  http://www.latticesemi.com/~/media/Documents/UserManuals/EI/icestickusermanual.pdf
 *  http://www.micron.com/~/media/documents/products/data-sheet/nor-flash/serial-nor/n25q/n25q_32mb_3v_65nm.pdf
 *  https://www.olimex.com/iCE40HX1K-EVB
 *
 *  VS2012 Release notes:
 *  ---------------------
 *  V1.00 : Aug,11,2017	- Benoit BOUCHEZ (BEB)
 *		- project ported to Visual Studio 2012
 *		- main source code turned into C++ rather than C to allow native bool support
 *		- deleted unused or non existing libraries for Windows (unistd, stdbool, etc...)
 *		- deleted GCC specific pragmas
 *		- new help display (simpler to read)
 *		- code presentation cleaned (removed extra spaces, better indentation, etc...)
 *		- getopt.c and getopt.h added to project (https://gist.github.com/ashelly/7776712)
 *		- better error handling to avoid endless "error" strings being displayed in many cases
 *		- mode variable deactivated in Bulk Erase command and correct detection of feedback from Arduino/Galileo
 *		- frames of more than 255 bytes are sent now in two times (Intel Galileo platform can not receive more than 255 bytes in a row)
 *        (delay of 20 milliseconds added between the two segments)
 */

#define READ_ID 	0x9F
#define PWR_UP 		0xAB
#define PWR_DOWN 	0xB9
#define WR_EN		0x06
#define BULK_ERASE	0xC7
#define SEC_ERASE	0xd8
#define PROG		0x02
#define READ		0x03
#define READY		0x44
#define READ_ALL	0x83
#define EMPTY		0x45

#include "stdafx.h"
#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
//#include <stdbool.h>		// BEB : removed non existing file for Windows platform
#include <stdlib.h>
//#include <unistd.h>		// BEB : removed non existing file for Windows platform
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <windows.h>
#include <sys/stat.h>
#include <direct.h>
#include "getopt.h"

// BEB : removed unknown pragma for Visual C++
//#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

DWORD bytes_written;
DWORD bytes_read;
uint32_t to=0;
int x;
uint8_t buffer_file[0x200000];

void serial_read(int x);

#define FEND	0xc0
#define FESC	0xdb
#define TFEND 	0xdc
#define TFESC	0xdd
bool newframe;
bool timeout;
bool escaped;
bool dont_erase = false;
bool mode = false;				// BEB : I hate those meaningless global variables... (see bugs found below)

#define SerialPort  "COM24"
uint8_t rxframe[512], txframe[512], fcs, tfcs;
uint16_t txp,rxp;
uint8_t membuf[0x200000];
uint8_t pages[0x2000];

bool verbose = false;

// Windows related serial port structures
HANDLE hSerial;
DCB dcbSerialParams = {0};
COMMTIMEOUTS timeouts = {0};
COMMPROP commProp = {0};
int sermemsize = 8192;		// Buffer size

int set_interface_attribs (char *pname)
{
	hSerial = CreateFile(pname, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
	if (hSerial == INVALID_HANDLE_VALUE)
	{
		fprintf(stderr,"Error : serial port does not exist or is not available (invalid file handle)\n");
		return -1;
	}
    
	SetupComm (hSerial, sermemsize, sermemsize);  /* Set buffer size. */
	PurgeComm (hSerial, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);
    
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (GetCommState(hSerial, &dcbSerialParams) == 0)
	{
		fprintf(stderr, "Error getting serial port state\n");
		CloseHandle(hSerial);
		return -1;
	}
     
	fprintf (stderr, "IMPORTANT : make sure that your Arduino script is compiled for 57600 bauds!\n");
	dcbSerialParams.BaudRate = CBR_57600;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if(SetCommState(hSerial, &dcbSerialParams) == 0)
	{
		fprintf(stderr, "Error setting serial port parameters\n");
		CloseHandle(hSerial);
		return -1;
	}

	memset ((void *) &timeouts, 0, sizeof (timeouts));
	timeouts.ReadIntervalTimeout = 1000;
	timeouts.ReadTotalTimeoutConstant = 2;
	timeouts.ReadTotalTimeoutMultiplier = 1;
	timeouts.WriteTotalTimeoutConstant = 2;
	timeouts.WriteTotalTimeoutMultiplier = 1;    

	if(SetCommTimeouts(hSerial, &timeouts) == 0)
	{
		fprintf(stderr, "Error setting serial port timeouts\n");
		CloseHandle(hSerial);
		return -1;
	}

	memset ((void *) &commProp, 0, sizeof (commProp));

	return 0;
}  // set_interface_attribs
// ----------------------------------------------

void usleep(unsigned long int val)
{
    LARGE_INTEGER ticksPerSecond;
	LARGE_INTEGER tick;
	LARGE_INTEGER tick_n;
	QueryPerformanceFrequency(&ticksPerSecond);
	QueryPerformanceCounter(&tick);
	val = (float)val * ((float)ticksPerSecond.QuadPart / (float)1000000);
	while(1)
	{
		QueryPerformanceCounter(&tick_n);
		if((tick_n.QuadPart - tick.QuadPart) > val)
		{
			break;
		}
	}
}  // usleep
// ----------------------------------------------

void startframe(uint8_t command)
{
	txframe[0]=FEND;
	txframe[1]=command;
	tfcs = command;
	txp = 2;
}  // startframe
// ----------------------------------------------

void addbyte(uint8_t newbyte)
{
	tfcs+=newbyte;

	switch(newbyte)
	{
		case FEND:
		txframe[txp++] = FESC;
		txframe[txp++] = TFEND;
		break;
		case FESC:
		txframe[txp++] = FESC;
		txframe[txp++] = TFESC;
		break; 

		default:
		txframe[txp++] = newbyte;
		break;
	}
}  // addbyte
// ----------------------------------------------

void sendframe()
{
	int SecondBlockSize;

	tfcs=0xff-tfcs;
	addbyte(tfcs);	
	txframe[txp++] = FEND;
#ifdef frdebug  
	fprintf(stderr, "%02X-L:%04X/%02X\n",txframe[txp-1],txp,tfcs);
#endif 

	SecondBlockSize=0;
	// Special handling for Intel Galileo boards
	// We have found that Galileo Gen 2 (maybe Gen 1 too) can't receive more than 255 bytes in one packet
	// If packet is more than 255 bytes, we split it in two, with a small delay between the two, to let Galileo process the first packet
	if (txp>255)
	{
		SecondBlockSize=txp-255;
		txp=255;
	}

	if(!WriteFile(hSerial, txframe, txp, &bytes_written, NULL))
	{
		fprintf(stderr, "Error\n");
		CloseHandle(hSerial);
		return;
	}  

	Sleep (20);		// Give time to Galileo to process the previous packet
	if (SecondBlockSize>0)
	{
		if(!WriteFile(hSerial, &txframe[255], SecondBlockSize, &bytes_written, NULL))
		{
			fprintf(stderr, "Error\n");
			CloseHandle(hSerial);
			return;
		}
	}  
}  // sendframe
// ----------------------------------------------

void error()
{
	exit(1);
}  // error
// ----------------------------------------------

bool waitframe(uint8_t cmd)
{
	rxp = 0;
	newframe = false;
	fcs = 0;
	uint32_t addr;

	for (to=0;to<5000;to++)
	{
		serial_read(0);
		usleep(500);			

		if (newframe)
		{
			rxp = 0;
			newframe = false;
			usleep(100);

	     	if ((rxframe[0]==READ_ID) && (cmd == READ_ID))
			{
				if (rxframe[1]==0xef) fprintf(stderr, "Winbond Serial Flash ");
				else fprintf(stderr, "Manufacturer ID: 0x%02X",rxframe[1]);
				if ((rxframe[2]==0x40) && (rxframe[3]==0x15)) fprintf(stderr, "- W25Q16BV\n");  
				else fprintf(stderr, " / Device ID: 0x%02X%02X\n",rxframe[2],rxframe[3]);
						
				return true;
			}	 
	     
			if (rxframe[0]==READ)
	     	{
				addr = (rxframe[1]<<16) | (rxframe[2]<<8);
				pages[addr>>8]=0;

				if (verbose)
				{
					fprintf (stderr, "READ %02X\n", addr);
				}
	             
				for (x=3; x < 259;x++)
				{
					membuf[addr++]=rxframe[x];
				}
			}

			if (rxframe[0]==READY)  
			{
				if (cmd==READY) return true; 
				// BEB : new handling of feedback from Arduino 
				if ((cmd==BULK_ERASE)&&(rxframe[1]==BULK_ERASE)) return true;
				to = 0;
			}
  
			if (rxframe[0]==EMPTY) 
			{
				if (verbose)
				{
					addr = (rxframe[1]<<16) | (rxframe[2]<<8);
					fprintf (stderr, "EMPTY %02X\n", addr);
				}

				pages[(rxframe[1]<<8) | rxframe[2]]=0;
			}
	
			if ((rxframe[0]==READ)||(rxframe[0]==EMPTY)) 
			{
				if (cmd==READ) return true; 
				else to = 0;
			}
			rxframe[0]=0;
			fcs = 0;
		}

	}
	return false;
}  // waitframe
// ----------------------------------------------
 
void flash_read_id()
{
	startframe(READ_ID);
	sendframe();
	waitframe(READ_ID);
	return;
 	
	fprintf(stderr, "\n");
}  // flash_read_id
// ----------------------------------------------

void flash_bulk_erase()
{
	bool Result;

	fprintf(stderr, "\nRequesting chip bulk erase..\n");
    //mode = true;			// BEB : bug corrected, application was continuing without waiting from acknowledge
	startframe(BULK_ERASE);
	sendframe();
    Result=waitframe(BULK_ERASE);
	if (Result==false) fprintf (stderr, "No answer from programmer!\n");
    else fprintf(stderr, "Chip erased\n");
    //mode = false;			// BEB
}  // flash_bulk_erase
// ----------------------------------------------

void flash_64kB_sector_erase(int addr)
{
	fprintf(stderr, "Erase 64kB sector at 0x%06X..", addr);
   //mode = true;
	startframe(SEC_ERASE);
	addbyte(addr>>8);
	addbyte(addr>>0);
	sendframe();
    waitframe(READY);
    fprintf(stderr, "Erased\n");
    //mode = false;
}  // flash_64kB_sector_erase
// ----------------------------------------------
 
void flash_read_all(void)
{
	bool notready=false;
	int s = 0;
	uint32_t i;

	fprintf(stderr, "Read 0x%06X +0x%06X..\n", 0, 0x200000);
	memset(&membuf,0xff,sizeof(membuf));
	memset(&pages,0xff,sizeof(pages));
    startframe(READ_ALL);
    sendframe();
	if (!waitframe(READY))
	{
		fprintf (stderr, "An error occured during memory dump...\n");
		error(); 
	}	 
	 
	fprintf(stderr, "Dump is finished. Now checking for mismatched frames...");
	while (1)
	{
		notready = false;
		for(x = 0; x < 0x2000;x++)
		{
			if (pages[x]>0)
			{
				if (s++ > 25)
				{
					fprintf(stderr, "ERROR Reading..\n");
					notready = false;
					break;
				} 
				notready=true;	
				fprintf(stderr, ".");
				startframe(READ);
				addbyte(x >> 8);
				addbyte(x >> 0);
				sendframe();
				waitframe(READ);
			}
		}
		if (!notready) break;
	}
	fprintf(stderr, "\n");

	if (verbose)
	{
		fprintf (stderr, "Memory content : \n");
		for (i = 0; i < 1024; i++)
			fprintf(stderr, "%02x%c", membuf[i], i == 0 || i % 32 == 31 ? '\n' : ' ');
	}
}  // flash_read_all
// ----------------------------------------------

void help(const char *progname)
{
	fprintf(stderr, "WinICEProg -- simple programming tool for Olimexino-32U4 based Lattice iCE programmers\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: %s [options] <filename>\n", progname);
	fprintf(stderr, "\n");
	fprintf(stderr, "-e : erase Flash memory\n");
	fprintf(stderr, "-w : write only, do not verify\n");
	fprintf(stderr, "-f : write all data, including 0xFF's pages\n");	
	fprintf(stderr, "-I<serialport> : port to connect Arduino\n");	
	fprintf(stderr, "-r : read entire flash (2MB) and write to file\n");
	fprintf(stderr, "-c : do not write flash, only verify (check)\n");
	fprintf(stderr, "-b : bulk erase entire flash before writing\n");
	fprintf(stderr, "-n : do not erase flash before writing\n");
	fprintf(stderr, "-t : just read the flash ID sequence\n");
	fprintf(stderr, "-v : verbose output\n");
	fprintf(stderr, "\n");
	exit(1);
}  // help
// ----------------------------------------------
 
void serial_read(int x)
{
	//DWORD dwCommEvent;
   	DWORD NumBytesReceived;
	
	while (1)
	{
		if (rxp>512)
		{
			newframe = 0;	
			rxp = 0;
			escaped = false;
			fcs = 0;	
			break;
		}	

		usleep(5);
		uint8_t buf[1];
		NumBytesReceived = 0;

		//n=read(fd, buf, 1);
		if(mode)
		{
    		while(NumBytesReceived == 1)
    		{
				if (!ReadFile(hSerial, buf, 1, &NumBytesReceived, NULL))
				{
					fprintf(stderr, "Serial port error\n");
					CloseHandle(hSerial);
					return;
				}
    		}
		}
		else
		{
			if (!ReadFile(hSerial, buf, 1, &NumBytesReceived, NULL))
			{
				fprintf(stderr, "Serial port error\n");
				CloseHandle(hSerial);
				return;
			}
		}

		// We received a byte on serial port
		if (NumBytesReceived==1)
		{		
			switch(buf[0])
			{
			case FEND:
				if (fcs==0xff)
				{
					newframe = 1;
					return;
				}
				newframe = 0;	
				rxp = 0;
				escaped = false;
				fcs = 0;
				break;
		
			case FESC:
				escaped = true;
				break;
		
			case TFESC:
				if (escaped)
				{
					rxframe[rxp++]=FESC;
					fcs+=FESC;
					escaped = false;
				}
				else 
				{
					rxframe[rxp++]=TFESC;
					fcs += TFESC;
				}
				break;
		
			case TFEND:
				if (escaped)
				{
					rxframe[rxp++]=FEND;
					fcs += FEND;
					escaped=false;
				}
				else
				{
					rxframe[rxp++]=TFEND;
					fcs += TFEND;
				}
				break;
		
			default:	
				escaped=false;
				rxframe[rxp++]=buf[0];
				fcs+=buf[0];
				break;
			}	
		}
		else break;
	}	
}  // serial_read
// ----------------------------------------------

int _tmain(int argc, _TCHAR* argv[])
{
	int addr;
	int max_read_size = 0x200000;
	bool read_mode = false;
	bool check_mode = false;
	bool bulk_erase = false;
	bool prog_sram = false;
	bool test_mode = false;
	bool noverify = true; // verify performs by arduino board
	bool ff_mode = false;
	bool bulk_erase_only = false;
	const char *filename = NULL;
	const char *devstr = NULL;
	int opt;
	char *portname = SerialPort;
	int errcode;

	// Browse all arguments in command line
	while ((opt = getopt(argc, argv, "I:rcbntvwfeh")) != -1)
	{
		switch (opt)
		{
		case 'd':
			devstr = optarg;
			break;
		case 'I':
			portname = optarg;
			break;
		case 'r':
			read_mode = true;
			noverify=false;
			break;
		case 'e':
			bulk_erase_only = true;
			break;			
		case 'f':
			ff_mode = true;
			break;			
		case 'R':
			read_mode = true;
			max_read_size = 256 * 1024;
			break;
		case 'c':
			check_mode = true;
			noverify=false;
			break;
		case 'b':
			bulk_erase = true;
			break;
		case 'n':
			dont_erase = true;
			break;
		case 'S':
			prog_sram = true;
			break;
		case 't':
			test_mode = true;
			printf("\nTest mode\n");
			break;
		case 'v':
			verbose = true;
			break;
		case 'w':
			noverify = true;
			break;			
		default:
			help(argv[0]);
		}
	}

	// Check that argument combination is valid
	if (read_mode + check_mode + prog_sram + test_mode > 1)
		help(argv[0]);

	if (bulk_erase && dont_erase)
		help(argv[0]);

	if (optind+1 != argc && !test_mode && !bulk_erase_only)
		help(argv[0]);

	filename = argv[optind];

	bulk_erase = true;  // comment this line if you do not want to bulk erase by default
			   
	// Activate serial port
	errcode=set_interface_attribs (portname);  // set speed to 57600 bps, 8n1 (no parity)
	// Stop now if serial port could not be opened
	if (errcode!=0) error();		

	usleep(100000);
	PurgeComm(hSerial, PURGE_RXCLEAR);
	PurgeComm(hSerial, PURGE_TXCLEAR);

    fprintf(stderr,"Serial: %s: %s\n",  portname, strerror (errno));
 
	// Execute commands depending on the flags
	rxp = 0;
	
	if (test_mode)
	{
		flash_read_id();
	}
	else if (prog_sram)
	{
		fprintf(stderr, "\nprogramming SRAM is not supported!\n");
	}
	else
	{
		// ---------------------------------------------------------
		// Program
		// ---------------------------------------------------------

		if (!read_mode && !check_mode && !bulk_erase_only)
		{
			
			FILE *f = (strcmp(filename, "-") == 0) ? stdin : fopen(filename, "rb");
			// Fixed name for debug phases
			//FILE* f=fopen("C:\\Users\\Benoit\\Desktop\\Raccourcis\\FPGA\\WinICEProg\\Debug\\top_bitmap.bin", "rb");	
			if (f == NULL) 
			{
				fprintf(stderr, "Error: Can't open '%s' for reading: %s\n", filename, strerror(errno));
				error();
			}

			if (!dont_erase)
			{
				// Full chip erase
				if (bulk_erase)
				{
					flash_bulk_erase();
				    //usleep(1000000);			// No need to wait, function returns only after programmer confirms that Flash has been erased
				}
				else
				{
					// Erase only sectors with data from file
					struct stat st_buf;
					if (stat(filename, &st_buf)) 
					{
						fprintf(stderr, "Error: Can't stat '%s': %s\n", filename, strerror(errno));
						error();
					}

					fprintf(stderr, "file size: %d\n", (int)st_buf.st_size);
					for (addr = 0; addr < st_buf.st_size; addr += 0x10000) 
					{
						flash_64kB_sector_erase(addr);	
					}
				}
			}
			else fprintf (stderr, "WARNING : Flash chip has not been erased!\n");

			flash_read_id();
			fprintf(stderr, "Programming..\n");
			int ccc;	

			PurgeComm(hSerial, PURGE_RXCLEAR);
			PurgeComm(hSerial, PURGE_TXCLEAR);

			for (addr = 0; true; addr += 256) 
			{
				uint8_t buffer[256];
				int rc = fread(buffer, 1, 256, f);

				if (rc <= 0) break;
				if (verbose)
					fprintf(stderr, "prog 0x%06X +0x%03X..\n", addr, rc);
				else
					fprintf(stderr, "\rprog 0x%06X +0x%03X..", addr, rc);
				for (ccc=0;ccc<rc;ccc++)
				{
					if ((buffer[ccc] != 0xFF) || (ff_mode))
					{
						int x;
						
						while(1)
						{	
							startframe(PROG);
							addbyte(addr>>16);
							addbyte(addr>>8);
							for (x=0;x<rc;x++)
								addbyte(buffer[x]);
							sendframe();
							
							if (waitframe(READY)) 
							{
								if (verbose)
									fprintf(stderr, ".");
								break;
							}
							fprintf (stderr, "Error : No answer from programmer!\n");
						}
						break;
					}	
				}			
			}
			fprintf(stderr, "\n");
 

			if (f != stdin)
				fclose(f);
		}

        if (!noverify && !bulk_erase_only)
        {
			// ---------------------------------------------------------
			// Read/Verify
			// ---------------------------------------------------------

			if (read_mode)
			{
				FILE *f = (strcmp(filename, "-") == 0) ? stdout :
					fopen(filename, "wb");
				if (f == NULL) {
					fprintf(stderr, "Error: Can't open '%s' for writing: %s\n", filename, strerror(errno));
					error();
				}

				fprintf(stderr, "\nreading..\n");
				flash_read_id();
				flash_read_all();
				printf("\nWriting data to file %s\n",filename);
				fwrite(membuf, 0x200000, 1, f);
 
				if (f != stdout)
					fclose(f);
			}
			else
			{
				FILE *f = (strcmp(filename, "-") == 0) ? stdin :
					fopen(filename, "rb");
				if (f == NULL) {
					fprintf(stderr, "Error: Can't open '%s' for reading: %s\n", filename, strerror(errno));
					error();
				}

				fprintf(stderr, "\nreading..\n");
 
				flash_read_id();
				flash_read_all();
				int rc = fread(buffer_file, 1, 0x200000, f);
				
				if (memcmp(buffer_file, membuf, rc)) 
				{
					fprintf(stderr, "Found difference between flash and file!\n");
					error();
				}

				fprintf(stderr, "\nVERIFY OK\n");

				if (f != stdin)
					fclose(f);
			}
		}

		if (bulk_erase_only)
		{
			long Debut=timeGetTime();
 			flash_bulk_erase();			
			fprintf (stderr, "%d\n", timeGetTime()-Debut);
			fprintf(stderr, "Flash erased!\n");
		}
		// ---------------------------------------------------------
		// Reset
		// ---------------------------------------------------------
	}

	// ---------------------------------------------------------
	// Exit
	// ---------------------------------------------------------
	fprintf(stderr, "Bye.\n");
    CloseHandle(hSerial); 
	return 0;
}  // main
// ----------------------------------------------

