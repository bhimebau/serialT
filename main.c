/* main.c ---
 *
 * Filename: main.c
 * Description:
 * Author: Bryce Himebaugh
  *
 */

/* Commentary:
 *
 *
 *
 */

/* Change log:
 *
 *
 */

/* Copyright 2013 Bryce Himebaugh
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/* Code: */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>
#include <string.h>

#define CHAR_DELAY_uS  1000                            // delay between character transmission
#define SELECT_TIMEOUT 1500000    	   	       	       // select cmd timeout 1.5s (number in uS)
// #define BAUD B115200
// #define SERIAL_PORT "/dev/ttyUSB0"

int open_serial_port(char *, int);
void print_help (void);

int main(int argc, char *argv[]) {
  int tty_handle;
  unsigned char ch;
  int n;
  fd_set readfds;
  struct timeval to;
  int select_status;

  struct termios old_tio, new_tio;

  const char port_str[]="-p";
  const char baud_str[]="-b";
  const char help_str[]="-h";
  const char buffer_str[]="-n";

  unsigned int baudrate = 9600;
  // unsigned int baudrate = 19200;
  // unsigned int baudtemp;
  char serialport[80] = "/dev/ttyUSB0";

  int i;
  int status;

  tcgetattr(STDIN_FILENO,&old_tio);
  new_tio=old_tio;
  new_tio.c_lflag &= ~ECHO;
  new_tio.c_lflag &= ~ICANON;
//  tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);


  //  printf("SerialT Startup\n");
  for (i=1;i<argc;i++) {
    if (!strcmp(argv[i],help_str)) {
      print_help();
      exit(0);
    }
    else if (!strcmp(argv[i],port_str)) {
      sprintf(serialport,"%s",argv[i+1]);
      i++;
    }
    else if (!strcmp(argv[i],baud_str)) {
 //     printf("Found Baud String, %s\n",argv[i+1]);
      baudrate = atoi(argv[i+1]);
//      printf("Baudtemp=%d\n",baudtemp);
//      if ((baudtemp) && !(baudtemp%1200)) {
 //       baudrate=baudtemp;
 //     printf("%d\n",baudrate);
 //     }
 //     else {
 //       printf("Error: Invalid baudrate %s\n",argv[i+1]);
 //       exit(0);
 //     }
      i++;
    }
    else if (!strcmp(argv[i],buffer_str)) {
//      setvbuf(stdin, NULL, _IONBF, 0);
//      setvbuf(stdout, NULL, _IONBF, 0);
//      setvbuf(stderr, NULL, _IONBF, 0);
      // adjust STDIN to be non-blocking, no echo
//      new_tio.c_lflag &= ~ECHO;
      new_tio.c_lflag |= ICANON;
      //  new_tio.c_lflag &=(~ICANON);
//      tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);
    }
    else {
      printf("Error: bad option %s\n",argv[i]);
      printf("\"SerialT -h\" for list of options\n");
      exit(0);
    }
  }
  //  printf("%s %d\n",serialport, baudrate);
  //  printf("Baudrate = %d\n",baudrate);
  if ((tty_handle = open_serial_port(serialport,baudrate)) == -1) {
    printf ("Error -> Could not open %s\n",serialport);
    exit(1);
  }

  tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);


  while (1) {
    // Select Command Preparation
    FD_ZERO(&readfds);                       // clear the read file descriptor set g
    FD_SET(STDIN_FILENO,&readfds);           // listen for stdin in the read desciptor set
    FD_SET(tty_handle,&readfds);             // listen for stdin in the read desciptor set
    to.tv_sec = SELECT_TIMEOUT/1000000;      // timeout seconds  = 1s (total timeout = 1.5uS)
    to.tv_usec = SELECT_TIMEOUT%1000000;     // timeout usec = .5s

    // wait here until one of the selectors becomes active
    select_status=select(tty_handle+1,&readfds,(fd_set *)0,(fd_set *)0,&to);
    if (select_status==-1);                  // Select Error
    else if (select_status==0);              // Select Timeout
    else {                                   // Descriptor is ready for one of the members
      // Check to see which descriptor is active
      // STDIN Handler
      if (FD_ISSET(STDIN_FILENO,&readfds)) {
        if ((n=read(STDIN_FILENO,&ch,1))!=0) {
	  //	  printf("pulled %c chars from STDIN\n",ch);
          status=write(tty_handle,&ch,1);     // send the character out the tty port
	  if (status!=1) {
	    printf("Error: Failed to write %c\n",ch);
	  }
          tcdrain(tty_handle);                // wait for the character to go
          usleep(CHAR_DELAY_uS);              // delay between characters, give goofy some time to process chars
        }
      }
      // Serial Port Handler
      if (FD_ISSET(tty_handle,&readfds)) {
        if ((n=read(tty_handle,&ch,1))!=0) {  // grab the data from the tty
	  //	  printf("%d\n",n);
          if ((ch>=' ')&&(ch<='~')){
            printf("%c",ch);
          }
          else if ((ch==0x0d)||(ch==0x0a)) {
            printf("\n");
          }
          else {
            printf("\\x%02x",ch);
          }
          fflush(stdout);
        }
      }
    }
  }
  tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);
}

int open_serial_port(char *port_string, int baud) {
  struct termios options;
  int fd;
  fd = open(port_string,O_RDWR|O_NOCTTY|O_NDELAY);
  if (fd == -1) {
    printf("Could Not Open %s\n",port_string);
    return (-1);
  }
  //  printf("opened the port");

  tcgetattr(fd,&options);
  //   memcpy(&tio, &_termios, sizeof(struct termios));
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  //  options.c_cflag = B115200 | CLOCAL | CREAD | CS8;
//   options.c_cflag = B9600 | CLOCAL | CREAD | CS8;
  options.c_lflag = 0;

  switch (baud) {
    case 50:
      options.c_cflag = B50 | CLOCAL | CREAD | CS8;
      break;
    case 75:
      options.c_cflag = B75 | CLOCAL | CREAD | CS8;
      break;
    case 110:
      options.c_cflag = B110 | CLOCAL | CREAD | CS8;
      break;
    case 134:
      options.c_cflag = B134 | CLOCAL | CREAD | CS8;
      break;
    case 150:
      options.c_cflag = B150 | CLOCAL | CREAD | CS8;
      break;
    case 200:
      options.c_cflag = B200 | CLOCAL | CREAD | CS8;
      break;
    case 300:
      options.c_cflag = B300 | CLOCAL | CREAD | CS8;
      break;
    case 600:
      options.c_cflag = B600 | CLOCAL | CREAD | CS8;
      break;
    case 1200:
      options.c_cflag = B1200 | CLOCAL | CREAD | CS8;
      break;
    case 1800:
      options.c_cflag = B1800 | CLOCAL | CREAD | CS8;
      break;
    case 2400:
      options.c_cflag = B2400 | CLOCAL | CREAD | CS8;
      break;
    case 4800:
      options.c_cflag = B4800 | CLOCAL | CREAD | CS8;
      break;
    case 9600:
      options.c_cflag = B9600 | CLOCAL | CREAD | CS8;
      break;
    case 19200:
      options.c_cflag = B19200 | CLOCAL | CREAD | CS8;
      break;
    case 38400:
      options.c_cflag = B38400 | CLOCAL | CREAD | CS8;
      break;
    case 57600:
      options.c_cflag = B57600 | CLOCAL | CREAD | CS8;
      break;
    case 115200:
      options.c_cflag = B115200 | CLOCAL | CREAD | CS8;
      break;
    case 230400:
      options.c_cflag = B230400 | CLOCAL | CREAD | CS8;
      break;
    case 460800:
      options.c_cflag = B460800 | CLOCAL | CREAD | CS8;
      break;
    case 500000:
      options.c_cflag = B500000 | CLOCAL | CREAD | CS8;
      break;
    case 576000:
      options.c_cflag = B576000 | CLOCAL | CREAD | CS8;
      break;
    case 921600:
      options.c_cflag = B921600 | CLOCAL | CREAD | CS8;
      break;
    case 1000000:
      options.c_cflag = B1000000 | CLOCAL | CREAD | CS8;
      break;
    case 1152000:
      options.c_cflag = B1152000 | CLOCAL | CREAD | CS8;
      break;
    case 1500000:
      options.c_cflag = B1500000 | CLOCAL | CREAD | CS8;
      break;
    case 2000000:
      options.c_cflag = B2000000 | CLOCAL | CREAD | CS8;
      break;
    case 2500000:
      options.c_cflag = B2500000 | CLOCAL | CREAD | CS8;
      break;
    case 3000000:
      options.c_cflag = B3000000 | CLOCAL | CREAD | CS8;
      break;
    case 3500000:
      options.c_cflag = B3500000 | CLOCAL | CREAD | CS8;
      break;
    case 4000000:
      options.c_cflag = B4000000 | CLOCAL | CREAD | CS8;
      break;
    default:
      printf("Error: Invalid Baud Rate, %d\n",baud);
      exit(1);
  }
  tcflush(fd, TCIFLUSH);

//  cfsetispeed(&options,baud);
//  cfsetospeed(&options,baud);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS;
  options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
  options.c_cflag |= (CLOCAL|CREAD);
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  tcsetattr(fd,TCSANOW,&options);
  return (fd);
}

void print_help (void) {
  printf("\nUSAGE: serialT [-h] [-p serial port device] [-b baud rate]\n");
  printf("-h Print this help message and exit\n");
  printf("-p Path to serial port device: default=/dev/ttyUSB0\n");
  printf("-b Baudrate of the serial port: default=9600\n");
  printf("-n disable buffered I/O\n");
}

