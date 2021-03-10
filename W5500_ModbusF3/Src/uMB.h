#include <stdint.h>
#include <stdio.h>

#define HEADER_START { 0x01, 0x00, 0x00 }

#define READ_COIL 0x01
#define READ_INPUT 0x02
#define READ_HOLDING_REGISTER 0x03
#define READ_INPUT_REGISTER 0x04
#define SET_COIL 0x05
#define SET_REGISTER 0x06
#define SET_COILS 0x0F
#define SET_REGISTERS 0x0

#ifndef _UMB_DEBUG
  #define _UMB_DEBUG 0x01
#endif

// Redefine for Your needs:
uint8_t message_req[128];
uint8_t message_resp[128];
uint16_t registers[64];

struct uMB_header{
  uint16_t tx;
  uint16_t prot;
  uint16_t len;
}; //__attribute__ ((packed))

struct uMB_request{
  struct uMB_header header;
  uint8_t unitID;
  uint8_t fcode;
  uint16_t address;
  uint16_t quantity;
  uint8_t *msg;
  uint8_t msg_len;
}; //__attribute__ ((packed))

struct uMB_response{
  struct uMB_header header;
  uint8_t unitID;
  uint8_t fcode;
  uint8_t quantity;
  uint16_t address;
  uint16_t *regs;
  uint8_t *msg;
  uint8_t msg_len;
}; //__attribute__ ((packed))

void word2bytes(uint16_t word, uint8_t* first, uint8_t* second);
void bytes2word(uint8_t first, uint8_t second, uint16_t* word);

void uMB_req_msg_create(struct uMB_request* req);
void uMB_req_msg_print(struct uMB_request* req);
void uMB_resp_msg_create(struct uMB_response* resp);
uint8_t uMB_check_id(uint8_t* msg_in, struct uMB_response* resp);
void uMB_resp_parse(uint8_t* msg_in, uint16_t data[], struct uMB_response* resp);
void uMB_resp_msg_print(struct uMB_response* resp);
