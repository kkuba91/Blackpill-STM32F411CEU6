# Blackpill-STM32F411CEU6 Wiznet W5500 Example
Blackpill STM32F411CEU6 board example start project with new addon (FREERTOS + W5500 TCP/IP server). 

Library used for W5500 from https://github.com/Wiznet/ioLibrary_Driver [1].

This is example of use W5500 created with STM32CubeIDE 1.3.0.

In the project used settings works with ST-Link V2 (mini) - SWD debugging.

The project (complete in .zip file) includes FREERTOS, OTG_USB and SPI (wiznet chip)

## Blackpill STM32F411 general settings in configurator
Same as in "ExapleProject".

## FREERTOS
Same as in "ExampleProject" (tasks: `ButtonTask`, `LedTask`, `USBTask`) + W5500 task: `SpiStartTask`.

All tasks run in the same priority without any data synchronization method.

## TCP SERVER
To check, how wiznet chip works as TCP server, there is written own loopback methon inside `SpiStartTask`. 
Especially connection abort procedure inside is impostant. The procedure shown in snippet below:

 ```c
...
getSn = getSn_SR(SockNr);
	  ir = Sn_IR(SockNr);
	  if(
      getSn == SOCK_CLOSED || getSn != SOCK_ESTABLISHED || 
      getSn == SOCK_CLOSE_WAIT || ir == Sn_IR_TIMEOUT || 
      sock_status == SOCKERR_TIMEOUT || phylink == PHY_LINK_OFF
      )
	  {
		  if(getSn == SOCK_CLOSE_WAIT || ir == Sn_IR_TIMEOUT || sock_status == SOCKERR_TIMEOUT) disconnect(SockNr);
		  close(SockNr);
		  socket(SockNr, Sn_MR_TCP, 502, SF_TCP_NODELAY);
		  tcp_sequence = 6;
	  }

	  getSn = getSn_SR(SockNr);
	  if(getSn == SOCK_INIT){
		  listen(SockNr);
		  tcp_sequence = 7;
	  }
...
```
The rest of the TCP server part is common for all manuals.

## MODBUS TCP CLIENT (DEMO - F3)
For tests there are prepared small library located inside `uMB.h` and `uMB.c`. 
It uses fast Modbus request construction and response preparation for external request message.

For use just define `request` and `response` structures. To have all interfaces even for Modbus Slave functionality, define both.

Example of use (with data from TCP/IP stock - W5500) below:
 ```c
...
if ( (rec_len = getSn_RX_RSR(SockNr) ) > 0){
		  rec_len = recv(SockNr, sock_buf, sock_len);
	  }
	  if(rec_len > 0)
	  {
		  request.msg = sock_buf;    //allocate request message as same as socket buffer
		  if(uMB_check_id(request.msg, &response)){   //Unit Id verification
		  	    uMB_resp_parse(request.msg, &hregisters[0], &response);
            //Here function out is response.msg and response.msg_len in general
		  	  }
		  sock_status = send(SockNr, response.msg, response.msg_len);    //Send response message
		  osDelay(10);
	  }
...
```


## Copywritghts
STM32, STM32Cube, STM32CubeIDE - ST Microelectronics company (C)

## References
[1] W5500 library https://github.com/Wiznet/ioLibrary_Driver
